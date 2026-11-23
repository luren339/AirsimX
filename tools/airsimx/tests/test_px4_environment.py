import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location('prepare_wsl', Path(__file__).resolve().parents[1] / 'px4/prepare_wsl.py')
prepare = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare)


class Px4EnvironmentTests(unittest.TestCase):
    def test_wsl_utf16_and_normal_utf8_are_decoded(self):
        self.assertEqual(prepare.decode('Ubuntu\r\n'.encode('utf-16-le')), 'Ubuntu\r\n')
        self.assertEqual(prepare.decode('verified'.encode()), 'verified')

    def test_msys_gpg_path_uses_drive_mount_syntax(self):
        if prepare.os.name != 'nt':
            self.skipTest('Windows drive conversion')
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            executable = folder / 'gpg.exe'
            (folder / 'msys-2.0.dll').write_bytes(b'')
            target = Path('D:/example/build/trust')
            self.assertEqual(prepare.gpg_path(target, executable), '/d/example/build/trust')
            (folder / 'msys-2.0.dll').unlink()
            self.assertEqual(prepare.gpg_path(target, executable), str(target.resolve()))

    def test_cached_file_never_downloaded_or_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'cached'
            target.write_bytes(b'original')
            with mock.patch.object(prepare.subprocess, 'run') as run:
                prepare.download('https://example.invalid/image', target, False)
            run.assert_not_called()
            self.assertEqual(target.read_bytes(), b'original')

    def test_offline_missing_file_has_no_network_side_effect(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(prepare.subprocess, 'run') as run:
                with self.assertRaises(FileNotFoundError):
                    prepare.download('https://example.invalid/image', Path(directory) / 'missing', True)
            run.assert_not_called()

    def test_partial_download_is_preserved_for_inspection(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'image.wsl'
            partial = target.with_suffix('.wsl.part')
            partial.write_bytes(b'partial')
            with mock.patch.object(prepare.subprocess, 'run') as run:
                with self.assertRaises(FileExistsError):
                    prepare.download('https://example.invalid/image', target, False)
            run.assert_not_called()
            self.assertEqual(partial.read_bytes(), b'partial')

    def test_failed_command_keeps_nonzero_exit_visible(self):
        result = subprocess.CompletedProcess(['wsl.exe'], 1, b'failed', b'detail')
        with mock.patch.object(prepare.subprocess, 'run', return_value=result):
            with self.assertRaisesRegex(RuntimeError, 'failed.*detail'):
                prepare.run(['wsl.exe'])

    def test_existing_distribution_refuses_reimport(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lock = json.loads(prepare.LOCK.read_text())
            with mock.patch.object(prepare, 'ROOT', root), \
                 mock.patch.object(prepare, 'run', return_value=(0, lock['wsl']['name'] + '\nUbuntu\n', '')) as run, \
                 mock.patch.object(prepare, 'download') as download:
                self.assertEqual(prepare.main(['import']), 1)
            self.assertEqual(run.call_count, 1)
            download.assert_not_called()
            records = list(root.rglob('result.json'))
            self.assertEqual(len(records), 1)
            self.assertEqual(json.loads(records[0].read_text())['status'], 'failed')


if __name__ == '__main__':
    unittest.main()
