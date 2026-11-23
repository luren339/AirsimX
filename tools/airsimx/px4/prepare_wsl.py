"""Download and verify a pinned Ubuntu image, then optionally import a new WSL2 distro."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
LOCK = Path(__file__).with_name('environment.lock.json')


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def decode(data):
    if b'\0' in data[:80]:
        return data.decode('utf-16-le', errors='replace').lstrip('﻿')
    return data.decode('utf-8-sig', errors='replace')


def run(command, timeout=120, check=True):
    result = subprocess.run([str(value) for value in command], cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout)
    output, error = decode(result.stdout), decode(result.stderr)
    if check and result.returncode:
        raise RuntimeError('Command failed (' + str(result.returncode) + '): ' + output + error)
    return result.returncode, output, error


def download(url, target, offline, timeout=300, resume=False):
    if target.is_file():
        return
    if offline:
        raise FileNotFoundError('Offline cache file missing: ' + str(target))
    partial = target.with_suffix(target.suffix + '.part')
    if partial.exists() and not resume:
        raise FileExistsError('Inspect the incomplete download before retrying: ' + str(partial))
    options = ['--continue-at', '-'] if partial.exists() else []
    result = subprocess.run(['curl.exe', '--fail', '--location', '--proto', '=https', '--tlsv1.2',
                             '--connect-timeout', '20', '--max-time', str(timeout), '--retry', '2',
                             '--retry-delay', '2', *options, '--output', str(partial), url], cwd=ROOT,
                            timeout=3 * timeout + 90)
    if result.returncode:
        raise RuntimeError('Download failed; incomplete file retained: ' + str(partial))
    partial.replace(target)


def gpg_path(path, executable):
    path = Path(path).resolve()
    if (Path(executable).parent / 'msys-2.0.dll').is_file():
        if len(path.drive) != 2 or path.drive[1] != ':':
            raise ValueError('MSYS GPG requires a local drive path')
        return '/' + path.drive[0].lower() + path.as_posix()[2:]
    return str(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('download', 'import', 'info'))
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--image-base-url', help='optional HTTPS image mirror; publisher signatures still come from Ubuntu')
    parser.add_argument('--resume-download', action='store_true', help='resume an inspected partial image, then verify the complete checksum')
    parser.add_argument('--environment-root', type=Path, default=ROOT.parent / 'AirsimX-Environment')
    args = parser.parse_args(argv)
    lock = json.loads(LOCK.read_text(encoding='utf-8'))
    ubuntu, wsl = lock['ubuntu'], lock['wsl']
    work = ROOT / 'build/airsimx/px4'
    output = work / 'records' / ('environment-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    record = {'command': args.command, 'status': 'running', 'lock_sha256': digest(LOCK), 'lock': lock}
    try:
        code, listing, unused = run(['wsl.exe', '--list', '--quiet'])
        distributions = [line.strip() for line in listing.splitlines() if line.strip()]
        record['distributions_before'] = distributions
        if args.command == 'info':
            record['wsl_list'] = run(['wsl.exe', '--list', '--verbose'])[1]
            record['status'] = 'passed'
            return 0
        if args.command == 'import' and wsl['name'] in distributions:
            raise RuntimeError('Distribution already exists; refusing to replace or re-register it')
        cache = ROOT / 'build/airsimx/downloads' / ('ubuntu-' + ubuntu['version'] + '-wsl-amd64')
        cache.mkdir(parents=True, exist_ok=True)
        sums, signature, key = cache / 'SHA256SUMS', cache / 'SHA256SUMS.gpg', cache / 'signing-key.asc'
        download(ubuntu['base_url'] + 'SHA256SUMS', sums, args.offline)
        download(ubuntu['base_url'] + 'SHA256SUMS.gpg', signature, args.offline)
        download(ubuntu['signing_key_url'], key, args.offline)
        gpg = shutil.which('gpg.exe') or shutil.which('gpg')
        if not gpg:
            raise RuntimeError('GPG is required to verify the Ubuntu release signature')
        trust = output / 'gnupg'
        trust.mkdir(mode=0o700)
        common = [gpg, '--batch', '--no-autostart', '--homedir', gpg_path(trust, gpg)]
        run(common + ['--import', gpg_path(key, gpg)])
        unused, verified, diagnostics = run(common + ['--status-fd', '1', '--verify', gpg_path(signature, gpg), gpg_path(sums, gpg)])
        fingerprints = [line.split()[2] for line in verified.splitlines() if line.startswith('[GNUPG:] VALIDSIG ')]
        if ubuntu['signing_key_fingerprint'] not in fingerprints:
            raise RuntimeError('Unexpected Ubuntu release signer: ' + repr(fingerprints))
        entries = dict((line.split()[1].lstrip('*'), line.split()[0]) for line in sums.read_text().splitlines() if line.strip())
        if entries.get(ubuntu['filename']) != ubuntu['sha256']:
            raise RuntimeError('Signed publisher checksum differs from the pinned image')
        image = cache / ubuntu['filename']
        image_base = args.image_base_url or ubuntu['base_url']
        if not image_base.startswith('https://') or '@' in image_base:
            raise ValueError('Image mirrors must use HTTPS without embedded credentials')
        image_url = image_base.rstrip('/') + '/' + ubuntu['filename']
        record['image_download_url'] = image_url
        download(image_url, image, args.offline, timeout=1800, resume=args.resume_download)
        if digest(image) != ubuntu['sha256']:
            raise RuntimeError('Ubuntu image checksum mismatch; image not imported')
        record.update(image=str(image), image_sha256=ubuntu['sha256'], verified_signers=fingerprints)
        if args.command == 'import':
            location = (args.environment_root / wsl['environment_subdirectory']).resolve()
            if location.is_relative_to(ROOT):
                raise ValueError('Keep the persistent WSL disk outside the Git checkout and disposable build directory')
            if location.exists():
                raise FileExistsError('Refusing to reuse an existing WSL installation directory: ' + str(location))
            location.parent.mkdir(parents=True, exist_ok=True)
            unused, output_text, error_text = run(['wsl.exe', '--import', wsl['name'], str(location),
                                                   str(image), '--version', str(wsl['version'])], timeout=900)
            record.update(install_location=str(location), import_output=output_text + error_text)
            after = [line.strip() for line in run(['wsl.exe', '--list', '--quiet'])[1].splitlines() if line.strip()]
            if wsl['name'] not in after or not set(distributions).issubset(after):
                raise RuntimeError('Unexpected distribution registration result')
            record['distributions_after'] = after
        record['status'] = 'passed'
        print(json.dumps({k: v for k, v in record.items() if k != 'lock'}, indent=2, ensure_ascii=True))
        return 0
    except Exception as error:
        record['status'] = 'failed'
        record['error'] = repr(error)
        print(str(error), file=sys.stderr)
        return 1
    finally:
        (output / 'result.json').write_text(json.dumps(record, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        print('Environment record: ' + str(output / 'result.json'), flush=True)


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    sys.exit(main())
