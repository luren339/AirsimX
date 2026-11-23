import argparse
import importlib.util
import json
from pathlib import Path
import stat
import tempfile
import unittest
import zipfile

SPEC = importlib.util.spec_from_file_location('build_windows', Path(__file__).resolve().parents[1] / 'build_windows.py')
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


class BuildToolTests(unittest.TestCase):
    def test_archive_traversal_and_symlinks_are_rejected(self):
        cases = ['../outside', '/absolute', 'C:/outside', 'folder\\outside', 'folder/../../outside']
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for i, name in enumerate(cases):
                archive = root / ('bad-' + str(i) + '.zip')
                with zipfile.ZipFile(archive, 'w') as package:
                    entry = zipfile.ZipInfo(name)
                    entry.filename = name
                    package.writestr(entry, 'bad')
                with self.assertRaises(ValueError, msg=name):
                    build.extract_checked(archive, root / ('out-' + str(i)))
            archive = root / 'link.zip'
            with zipfile.ZipFile(archive, 'w') as package:
                entry = zipfile.ZipInfo('link')
                entry.create_system = 3
                entry.external_attr = (stat.S_IFLNK | 0o777) << 16
                package.writestr(entry, '../outside')
            with self.assertRaises(ValueError):
                build.extract_checked(archive, root / 'link-out')

    def test_normal_archive_and_nonempty_destination(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'good.zip'
            with zipfile.ZipFile(archive, 'w') as package:
                package.writestr('source/include/header.h', 'content')
            output = root / 'output'
            build.extract_checked(archive, output)
            self.assertEqual((output / 'source/include/header.h').read_text(), 'content')
            with self.assertRaises(ValueError):
                build.extract_checked(archive, output)

    def test_case_insensitive_duplicate_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'duplicate.zip'
            with zipfile.ZipFile(archive, 'w') as package:
                package.writestr('source/File.h', 'one')
                package.writestr('source/file.h', 'two')
            with self.assertRaises(ValueError):
                build.extract_checked(archive, root / 'output')

    def test_cached_archive_is_checked_without_modification(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'dependency-1/source.zip'
            archive.parent.mkdir()
            archive.write_bytes(b'expected')
            entry = {'folder': 'dependency-1', 'sha256': build.sha256(archive)}
            self.assertEqual(build.obtain_archive(entry, root, True), archive)
            archive.write_bytes(b'changed')
            with self.assertRaises(ValueError):
                build.obtain_archive(entry, root, True)
            self.assertEqual(archive.read_bytes(), b'changed')
            with self.assertRaises(FileNotFoundError):
                build.obtain_archive({'folder': 'missing'}, root, True)

    def test_copy_preserves_unchanged_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, target = root / 'source', root / 'sub/target'
            source.write_text('value')
            build.copy_file(source, target)
            modified = target.stat().st_mtime_ns
            build.copy_file(source, target)
            self.assertEqual(target.stat().st_mtime_ns, modified)
            source.write_text('replacement')
            build.copy_file(source, target)
            self.assertEqual(target.read_text(), 'replacement')

    def test_batch_arguments_quote_spaces_and_reject_expansion(self):
        self.assertEqual(build.cmd_quote('D:/Program Files/tool.bat'), '"D:/Program Files/tool.bat"')
        self.assertEqual(build.cmd_quote('D:/a&b'), '"D:/a&b"')
        for value in ('%PATH%', 'a"b', 'a\nb', 'D:/Flight!PATH!Lab', 'D:/Flight!Lab'):
            with self.assertRaises(ValueError):
                build.cmd_quote(value)

    def test_output_directory_cannot_overwrite_previous_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'AirSim.props').write_text('')
            output = root / 'record'
            args = argparse.Namespace(root=root, cache_dir=None, project=None, command='prepare', output=output)
            first = build.Builder(args)
            self.assertEqual(json.loads((output / 'result.json').read_text())['status'], 'running')
            with self.assertRaises(FileExistsError):
                build.Builder(args)


if __name__ == '__main__':
    unittest.main()
