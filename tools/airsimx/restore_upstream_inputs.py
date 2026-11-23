"""Restore omitted native build inputs from a locally obtained, pinned Microsoft AirSim Git object tree."""
import argparse
import hashlib
from pathlib import Path, PurePosixPath
import subprocess

BASE = '1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41'
ROOT = Path(__file__).resolve().parents[2]
DIRECTORIES = ('MavLinkCom/mavlink', 'Unreal/Plugins/AirSim/Content', 'Unreal/Environments/Blocks/Content')
EXTENSIONS = {'.h', '.xml', '.uasset', '.umap', '.png'}


def git(source, *args):
    return subprocess.check_output(['git', '--no-replace-objects', '-C', str(source), *args])


def entries(source, folders):
    for record in git(source, 'ls-tree', '-rz', BASE, '--', *folders).split(b'\0'):
        if not record:
            continue
        meta, raw_path = record.split(b'\t', 1)
        mode, kind, oid = meta.split()
        name = PurePosixPath(raw_path.decode())
        if name.name == '.gitignore':
            continue
        if (mode != b'100644' or kind != b'blob' or name.suffix.lower() not in EXTENSIONS or
                not any(name.is_relative_to(d) for d in folders) or name.is_absolute() or '..' in name.parts):
            raise ValueError('unexpected input at pinned revision: ' + str(name))
        yield name, oid.decode()


def blob_id(data):
    return hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()


def restore(source, core_only):
    if git(source, 'rev-parse', '--verify', BASE + '^{commit}').decode().strip() != BASE:
        raise ValueError('upstream clone does not contain the pinned commit')
    inputs = list(entries(source, DIRECTORIES[:1] if core_only else DIRECTORIES))
    if not inputs:
        raise ValueError('no matching upstream inputs')
    new = []
    for path, oid in inputs:
        target = ROOT / path
        if target.resolve() != ROOT.resolve() / path or target.is_symlink():
            raise ValueError(f'refusing linked destination {path}')
        if target.exists():
            if not target.is_file() or blob_id(target.read_bytes()) != oid:
                raise FileExistsError(f'conflicting input, nothing overwritten: {path}')
        else:
            new.append((target, oid))
    for target, oid in new:
        data = git(source, 'cat-file', 'blob', oid)
        if blob_id(data) != oid:
            raise ValueError('input hash mismatch')
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as f:
            f.write(data)
    print(f'{len(inputs)} pinned inputs: {len(new)} restored, {len(inputs)-len(new)} already identical.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('upstream', type=Path, help='local Microsoft AirSim Git clone (no checkout needed)')
    p.add_argument('--core-only', action='store_true', help='only the generated protocol headers and XML, no UE content')
    a = p.parse_args()
    restore(a.upstream.resolve(), a.core_only)
