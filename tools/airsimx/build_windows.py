"""Reproducible, project-local Windows build steps; never installs system tools."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import time
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
LOCK = Path(__file__).with_name('dependencies.lock.json')


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def git_state(root):
    """Record which source the build came from, like the offline CI report.

    A build record without a commit cannot be matched to the tree that
    produced it later, so every record carries HEAD, branch and whether the
    working tree was dirty. Outside a git checkout the fields are null and
    the build still proceeds.
    """
    state = {'commit': None, 'branch': None, 'dirty': None}
    for key, arguments in (('commit', ('rev-parse', 'HEAD')),
                            ('branch', ('rev-parse', '--abbrev-ref', 'HEAD'))):
        try:
            result = subprocess.run(['git', *arguments], cwd=root, capture_output=True,
                                    text=True, timeout=30)
            if result.returncode == 0:
                state[key] = result.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    try:
        result = subprocess.run(['git', 'status', '--porcelain'], cwd=root,
                                capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            state['dirty'] = bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return state


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def copy_file(source, destination):
    if not source.is_file():
        raise FileNotFoundError(source)
    if destination.is_file() and sha256(source) == sha256(destination):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def copy_tree(source, destination):
    if not source.is_dir():
        raise FileNotFoundError(source)
    for path in sorted(source.rglob('*')):
        if path.is_symlink():
            raise ValueError('Symlink not supported in build inputs: ' + str(path))
        if path.is_file():
            copy_file(path, destination / path.relative_to(source))


def extract_checked(archive, destination):
    if destination.exists() and any(destination.iterdir()):
        raise ValueError('Extraction requires an empty directory')
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as package:
        seen = set()
        for member in package.infolist():
            name = member.orig_filename
            parts = PurePosixPath(name).parts
            if not parts or '\\' in name or ':' in name or '\x00' in name or name.startswith('/') or '..' in parts:
                raise ValueError('Unsafe archive path: ' + name)
            reserved = {'CON', 'PRN', 'AUX', 'NUL'} | {prefix + str(i) for prefix in ('COM', 'LPT') for i in range(1, 10)}
            if any(part.rstrip(' .') != part or part.split('.')[0].upper() in reserved for part in parts):
                raise ValueError('Unsafe Windows archive path: ' + name)
            if stat.S_ISLNK(member.external_attr >> 16):
                raise ValueError('Archive symlink rejected: ' + name)
            normalized = '/'.join(parts).lower()
            if normalized in seen:
                raise ValueError('Duplicate archive path: ' + name)
            seen.add(normalized)
            target = (destination / Path(*parts)).resolve()
            if not target.is_relative_to(destination.resolve()):
                raise ValueError('Archive path escapes destination: ' + name)
        if sum(m.file_size for m in package.infolist()) > 512 * 1024 * 1024:
            raise ValueError('Dependency archive exceeds extraction size limit')
        package.extractall(destination)


def obtain_archive(entry, cache, offline):
    target = cache / entry['folder'] / 'source.zip'
    if target.exists():
        if sha256(target) != entry['sha256']:
            raise ValueError('Cached archive checksum mismatch; left untouched: ' + str(target))
        return target
    if offline:
        raise FileNotFoundError('Offline archive missing: ' + str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix('.part')
    if partial.exists():
        raise FileExistsError('Incomplete download exists; inspect before removing: ' + str(partial))
    created = False
    try:
        request = urllib.request.Request(entry['url'], headers={'User-Agent': 'AirsimX-build/0.1'})
        deadline = time.monotonic() + 300
        with partial.open('xb') as output:
            created = True
            with urllib.request.urlopen(request, timeout=60) as response:
                if not response.url.startswith('https://'):
                    raise ValueError('Dependency download redirected away from HTTPS')
                total = 0
                while True:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Dependency download exceeded five minutes')
                    block = response.read1(65536)
                    if not block:
                        break
                    total += len(block)
                    if total > 128 * 1024 * 1024:
                        raise ValueError('Dependency download exceeds size limit')
                    output.write(block)
        if sha256(partial) != entry['sha256']:
            raise ValueError('Downloaded archive checksum mismatch: ' + entry['name'])
        partial.replace(target)
    except BaseException:
        if created:
            partial.unlink(missing_ok=True)
        raise
    return target


def cmd_quote(value):
    value = str(value)
    if any(char in value for char in ('"', '%', '!', '\n', '\r')):
        raise ValueError('Unsupported batch argument: ' + value)
    return '"' + value + '"'


def batch_command(path, args):
    command = 'call ' + ' '.join(cmd_quote(value) for value in (path, *args))
    return cmd_quote(os.environ.get('COMSPEC', 'cmd.exe')) + ' /d /s /c "' + command + '"'


def toolchain():
    if os.name != 'nt':
        raise RuntimeError('Native build commands require Windows')
    installer = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer'
    vswhere = installer / 'vswhere.exe'
    if not vswhere.is_file():
        raise FileNotFoundError('Install Visual Studio 2022 C++ Build Tools; vswhere.exe was not found')
    location = subprocess.check_output([str(vswhere), '-latest', '-version', '[17.0,18.0)', '-products', '*',
                                       '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
                                       '-property', 'installationPath'], text=True, timeout=30).strip()
    if not location:
        raise RuntimeError('Visual Studio 2022 C++ toolchain not found')
    vs = Path(location)
    vcvars = vs / 'VC/Auxiliary/Build/vcvars64.bat'
    command = 'call ' + cmd_quote(vcvars) + ' >nul && set'
    bootstrap = {key.upper(): value for key, value in os.environ.items()}
    bootstrap['PATH'] = str(installer) + os.pathsep + bootstrap.get('PATH', '')
    output = subprocess.check_output(cmd_quote(os.environ.get('COMSPEC', 'cmd.exe')) + ' /d /s /c "' + command + '"',
                                     env=bootstrap, text=True, errors='replace', timeout=60)
    env = dict(bootstrap)
    for line in output.splitlines():
        if '=' in line and not line.startswith('='):
            key, value = line.split('=', 1)
            env[key.upper()] = value
    cmake = vs / 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
    if not cmake.is_file():
        found = shutil.which('cmake', path=env.get('PATH'))
        if not found:
            raise FileNotFoundError('CMake not found; no automatic system installation is attempted')
        cmake = Path(found)
    env['PATH'] = str(installer) + os.pathsep + str(cmake.parent) + os.pathsep + env['PATH']
    return env, cmake, {'visual_studio': str(vs), 'vc_tools_version': env.get('VCTOOLSVERSION'),
                        'windows_sdk_version': env.get('WINDOWSSDKVERSION')}


class Builder:
    def __init__(self, args):
        self.args = args
        self.root = args.root.resolve()
        if not (self.root / 'AirSim.props').is_file():
            raise ValueError('Not an AirSim checkout: ' + str(self.root))
        self.work = self.root / 'build/airsimx'
        self.cache = (args.cache_dir or self.work / 'downloads').resolve()
        self.project = (args.project or self.root / 'Unreal/Environments/Blocks/Blocks.uproject').resolve()
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.output = (args.output or self.work / 'build-records' / (args.command + '-' + stamp)).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.record = {'command': args.command, 'root': str(self.root), 'status': 'running', 'steps': [],
                       'dependency_lock_sha256': sha256(LOCK), 'script_sha256': sha256(Path(__file__)),
                       'source': git_state(self.root), 'output': str(self.output)}
        self.env = dict(os.environ)
        self.cmake = None
        self.save()

    def save(self):
        write_json(self.output / 'result.json', self.record)

    def run(self, label, command, timeout=1800):
        print(label, flush=True)
        logfile = self.output / (label + '.log')
        invocation = command if isinstance(command, str) else [str(v) for v in command]
        with logfile.open('w', encoding='utf-8') as stream:
            process = subprocess.Popen(invocation, cwd=self.root, env=self.env,
                                       stdout=stream, stderr=subprocess.STDOUT)
            try:
                code = process.wait(timeout=timeout)
            except BaseException:
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
                else:
                    process.kill()
                process.wait(timeout=30)
                raise
        self.record['steps'].append({'name': label, 'command': invocation,
                                     'exit_code': code, 'log': str(logfile)})
        self.save()
        if code:
            raise RuntimeError(label + ' failed with exit ' + str(code) + '; see ' + str(logfile))

    def prepare(self):
        dependencies = json.loads(LOCK.read_text(encoding='utf-8'))['dependencies']
        extraction_root = self.work / 'extracted'
        extraction_root.mkdir(parents=True, exist_ok=True)
        for entry in dependencies:
            archive = obtain_archive(entry, self.cache, self.args.offline)
            with tempfile.TemporaryDirectory(prefix=entry['name'] + '-', dir=extraction_root) as temporary:
                extracted = Path(temporary)
                extract_checked(archive, extracted)
                source = extracted / entry['folder']
                if not (source / entry['license_file']).is_file():
                    raise ValueError('Unexpected dependency archive layout: ' + entry['name'])
                if entry['name'] == 'rpclib':
                    copy_tree(source, self.root / 'external/rpclib' / entry['folder'])
                    copy_tree(source / 'include', self.root / 'AirLib/deps/rpclib/include')
                    copy_file(source / entry['license_file'], self.root / 'AirLib/deps/rpclib' / entry['license_file'])
                else:
                    copy_tree(source / 'Eigen', self.root / 'AirLib/deps/eigen3/Eigen')
                    for license_file in source.glob('COPYING*'):
                        copy_file(license_file, self.root / 'AirLib/deps/eigen3' / license_file.name)
            print('Prepared ' + entry['name'] + ' ' + entry['version'], flush=True)
        write_json(self.work / 'prepared-dependencies.json', {'lock_sha256': sha256(LOCK), 'dependencies': dependencies})
        self.record['dependencies'] = dependencies

    def core(self):
        prepared = self.work / 'prepared-dependencies.json'
        if not prepared.is_file() or json.loads(prepared.read_text())['lock_sha256'] != sha256(LOCK):
            raise RuntimeError('Run the prepare command with the current dependency lock before building')
        self.env, self.cmake, self.record['toolchain'] = toolchain()
        rpc_source = self.root / 'external/rpclib/rpclib-2.3.0'
        rpc_build = self.work / 'rpclib'
        self.run('rpclib-configure', [self.cmake, '-S', rpc_source, '-B', rpc_build,
                                    '-G', 'Visual Studio 17 2022', '-A', 'x64',
                                    '-DRPCLIB_BUILD_TESTS=OFF', '-DRPCLIB_BUILD_EXAMPLES=OFF'])
        configs = ('Release', 'Debug') if self.args.configuration == 'Both' else (self.args.configuration,)
        for config in configs:
            self.run('rpclib-' + config, [self.cmake, '--build', rpc_build, '--config', config,
                                         '--parallel', str(self.args.jobs)])
            copy_file(rpc_build / config / 'rpc.lib', self.root / 'AirLib/deps/rpclib/lib/x64' / config / 'rpc.lib')
            self.run('core-' + config, [Path(self.record['toolchain']['visual_studio']) / 'MSBuild/Current/Bin/MSBuild.exe',
                                       self.root / 'AirLibUnitTests/AirLibUnitTests.vcxproj',
                                       '/p:Platform=x64', '/p:Configuration=' + config,
                                       '/m:' + str(self.args.jobs), '/nologo', '/verbosity:minimal'])
            self.run('tests-' + config, [self.root / 'AirLibUnitTests/build/x64' / config / 'AirLibUnitTests.exe',
                                        '--timeout-seconds', '60'], timeout=360)
            copy_file(self.root / 'MavLinkCom/lib/x64' / config / 'MavLinkCom.lib',
                      self.root / 'AirLib/deps/MavLinkCom/lib/x64' / config / 'MavLinkCom.lib')
        copy_tree(self.root / 'MavLinkCom/include', self.root / 'AirLib/deps/MavLinkCom/include')
        for license_file in (self.root / 'MavLinkCom').glob('LICENSE*'):
            copy_file(license_file, self.root / 'AirLib/deps/MavLinkCom' / license_file.name)

    def stage(self):
        template = self.root / 'Unreal/Plugins/AirSim'
        plugin = self.project.parent / 'Plugins/AirSim'
        if plugin.resolve() == template.resolve():
            raise ValueError('The project plugin must be separate from its source template')
        if self.args.configuration == 'Debug':
            raise ValueError('Development UE targets require Release libraries; stage Release or Both')
        generated = ('Binaries', 'Intermediate', 'Saved', 'DerivedDataCache')
        staging = self.project.parent / 'Saved/AirsimXStaging'
        staging.mkdir(parents=True, exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix='plugin-', dir=staging))
        marker = '.airsimx-plugin-stage.json'

        def managed_files(directory):
            result = {}
            for current, dirs, files in os.walk(directory):
                relative = Path(current).relative_to(directory)
                dirs[:] = [d for d in dirs if d not in generated and d != '.git']
                for name in files:
                    if relative == Path('.') and name == marker:
                        continue
                    path = Path(current) / name
                    result[(relative / name).as_posix()] = sha256(path)
            return result

        try:
            for directory, dirs, files in os.walk(template):
                relative = Path(directory).relative_to(template)
                dirs[:] = [d for d in dirs if d not in generated and d != '.git'
                           and not (relative == Path('Source') and d == 'AirLib')]
                for filename in files:
                    copy_file(Path(directory) / filename, temporary / relative / filename)
            bundle = temporary / 'Source/AirLib'
            copy_tree(self.root / 'AirLib/include', bundle / 'include')
            configs = ('Release', 'Debug') if self.args.configuration == 'Both' else ('Release',)
            for config in configs:
                copy_file(self.root / 'AirLib/lib/x64' / config / 'AirLib.lib', bundle / 'lib/x64' / config / 'AirLib.lib')
                for dependency, library in (('rpclib', 'rpc'), ('MavLinkCom', 'MavLinkCom')):
                    base = Path('deps') / dependency
                    copy_file(self.root / 'AirLib' / base / 'lib/x64' / config / (library + '.lib'),
                              bundle / base / 'lib/x64' / config / (library + '.lib'))
            for dependency in ('rpclib', 'MavLinkCom'):
                copy_tree(self.root / 'AirLib/deps' / dependency / 'include', bundle / 'deps' / dependency / 'include')
                for path in (self.root / 'AirLib/deps' / dependency).glob('LICENSE*'):
                    copy_file(path, bundle / 'deps' / dependency / path.name)
            copy_tree(self.root / 'AirLib/deps/eigen3/Eigen', bundle / 'deps/eigen3/Eigen')
            for path in (self.root / 'AirLib/deps/eigen3').glob('COPYING*'):
                copy_file(path, bundle / 'deps/eigen3' / path.name)
            copy_file(self.root / 'LICENSE', bundle / 'LICENSE')
            cpp_files = [p for p in bundle.rglob('*') if p.suffix.lower() in ('.cpp', '.cc', '.c', '.cxx')]
            if cpp_files:
                raise ValueError('Unexpected compilation units in staged header/static-library bundle: ' + str(cpp_files))
            bundle_manifest = {p.relative_to(bundle).as_posix(): sha256(p) for p in sorted(bundle.rglob('*')) if p.is_file()}
            write_json(bundle / '.airsimx-stage.json', {'dependency_lock_sha256': sha256(LOCK), 'files': bundle_manifest})
            owned_files = set(managed_files(temporary))
            previous_marker = plugin / marker
            previous_metadata = json.loads(previous_marker.read_text(encoding='utf-8')) if previous_marker.is_file() else {}
            previous_owned = set(previous_metadata.get('owned_files', []))
            installed_files = managed_files(plugin) if plugin.is_dir() else {}
            for name in installed_files:
                if name not in owned_files and name not in previous_owned and not name.startswith('Source/AirLib/'):
                    copy_file(plugin / name, temporary / name)
            manifest = managed_files(temporary)
            def native_files(files):
                return {name: checksum for name, checksum in files.items()
                        if name.startswith('Source/') or name.endswith('.uplugin')}
            current_format = previous_metadata.get('stage_format_version') == 2
            native_inputs_changed = (not current_format or native_files(manifest) != native_files(installed_files)
                                     or native_files(manifest) != native_files(previous_metadata.get('files', {})))
            unchanged = current_format and plugin.is_dir() and installed_files == manifest
            if not unchanged:
                # UBT considers copied object creation times; retaining Intermediate after a source change can hide that change.
                for directory in generated:
                    if directory == 'Intermediate' and native_inputs_changed:
                        continue
                    if (plugin / directory).is_dir():
                        copy_tree(plugin / directory, temporary / directory)
                write_json(temporary / marker, {'stage_format_version': 2, 'dependency_lock_sha256': sha256(LOCK),
                                               'files': manifest, 'owned_files': sorted(owned_files)})
                backup = None
                if plugin.exists():
                    backup_root = self.project.parent / 'Saved/AirsimXStagingBackups'
                    backup_root.mkdir(parents=True, exist_ok=True)
                    backup = backup_root / ('AirSim-' + self.output.name)
                    if backup.exists():
                        raise FileExistsError(backup)
                    plugin.replace(backup)
                plugin.parent.mkdir(parents=True, exist_ok=True)
                try:
                    temporary.replace(plugin)
                except BaseException:
                    if backup is not None:
                        backup.replace(plugin)
                    raise
                if backup is not None:
                    self.record['previous_plugin_backup'] = str(backup)
            self.record['staged_plugin'] = str(plugin)
            self.record['staged_file_count'] = len(manifest)
            self.record['staged_configurations'] = list(configs)
            self.record['stage_unchanged'] = unchanged
            self.record['native_inputs_changed'] = native_inputs_changed
            self.record['plugin_intermediate_reused'] = not native_inputs_changed and (plugin / 'Intermediate').is_dir()
            print('Plugin staged without third-party example sources: ' + str(plugin), flush=True)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    def target(self, kind):
        name = getattr(self.args, kind + '_target', None) or self.project.stem + ('Editor' if kind == 'editor' else '')
        if not name.isascii() or not name.isidentifier():
            raise ValueError('Invalid UE target name: ' + name)
        target_file = self.project.parent / 'Source' / (name + '.Target.cs')
        if not target_file.is_file():
            raise ValueError('UE target not found: ' + str(target_file) + '; pass --' + kind + '-target for a different C++ target name')
        self.record[kind + '_target'] = name
        return name

    def engine(self):
        engine = self.args.engine_root or (Path(os.environ['UE_ROOT']) if os.environ.get('UE_ROOT') else None)
        if engine is None:
            raise ValueError('Pass --engine-root or set UE_ROOT for this process; no registry/global configuration is changed')
        engine = engine.resolve()
        version = engine / 'Engine/Build/Build.version'
        if not version.is_file():
            raise FileNotFoundError(version)
        self.record['engine'] = json.loads(version.read_text())
        if (self.record['engine']['MajorVersion'], self.record['engine']['MinorVersion']) != (5, 8):
            raise ValueError('This baseline is validated for UE 5.8 only')
        return engine

    def editor(self):
        engine = self.engine()
        self.run('ue-editor', batch_command(engine / 'Engine/Build/BatchFiles/Build.bat',
                 [self.target('editor'), 'Win64', 'Development', '-Project=' + str(self.project),
                  '-WaitMutex', '-NoUBA', '-MaxParallelActions=' + str(self.args.jobs)]))
        dll = self.project.parent / 'Plugins/AirSim/Binaries/Win64/UnrealEditor-AirSim.dll'
        self.record['editor_plugin_sha256'] = sha256(dll)

    def package(self):
        engine = self.engine()
        archive = (self.args.archive_dir or self.work / 'packages' / self.output.name).resolve()
        if archive.exists():
            raise FileExistsError('Packaging requires a fresh archive directory: ' + str(archive))
        game_target = self.target('game')
        self.run('ue-game', batch_command(engine / 'Engine/Build/BatchFiles/Build.bat',
                 [game_target, 'Win64', 'Development', '-Project=' + str(self.project),
                  '-WaitMutex', '-NoUBA', '-MaxParallelActions=' + str(self.args.jobs)]), timeout=3600)
        self.run('ue-package', batch_command(engine / 'Engine/Build/BatchFiles/RunUAT.bat',
                 ['BuildCookRun', '-project=' + str(self.project), '-target=' + game_target, '-noP4', '-platform=Win64',
                  '-clientconfig=Development', '-build', '-cook', '-stage', '-pak', '-archive',
                  '-archivedirectory=' + str(archive), '-unattended', '-utf8output']), timeout=7200)
        executable_names = {game_target + '.exe'}
        for receipt in (self.project.parent / 'Binaries/Win64').glob(game_target + '*.target'):
            data = json.loads(receipt.read_text(encoding='utf-8'))
            if data.get('TargetName') != game_target or data.get('Configuration') != 'Development':
                continue
            executable_names.update(Path(product['Path']).name for product in data.get('BuildProducts', [])
                                    if product.get('Type') == 'Executable')
        executables = [path for path in archive.rglob('*.exe') if path.name in executable_names]
        if not executables:
            raise RuntimeError('UAT returned success but the packaged executable for ' + game_target + ' was not found')
        self.record['package_directory'] = str(archive)
        self.record['package_executables'] = {str(p.relative_to(archive)): sha256(p) for p in executables}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'core', 'stage', 'editor', 'package', 'all'))
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--engine-root', type=Path)
    parser.add_argument('--project', type=Path)
    parser.add_argument('--game-target', help='game Target.cs name, defaults to the project name')
    parser.add_argument('--editor-target', help='editor Target.cs name, defaults to project name plus Editor')
    parser.add_argument('--cache-dir', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--archive-dir', type=Path)
    parser.add_argument('--configuration', choices=('Release', 'Debug', 'Both'), default='Both')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args(argv)
    if args.jobs < 1 or args.jobs > 64:
        parser.error('--jobs must be between 1 and 64')
    if args.configuration == 'Debug' and args.command in ('stage', 'editor', 'package', 'all'):
        parser.error('Development UE targets require Release libraries; select Release or Both')
    builder = None
    try:
        builder = Builder(args)
        steps = ('prepare', 'core', 'stage', 'editor') if args.command == 'all' else (args.command,)
        for step in steps:
            getattr(builder, step)()
            builder.save()
        builder.record['status'] = 'passed'
        builder.save()
        print('PASS: ' + str(builder.output / 'result.json'), flush=True)
        return 0
    except Exception as error:
        if builder is not None:
            builder.record['status'] = 'failed'
            builder.record['error'] = str(error)
            builder.save()
        print('ERROR: ' + str(error), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
