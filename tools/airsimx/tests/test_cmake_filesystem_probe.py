"""Exercise the CMake filesystem link probe with real C++17 link attempts.

Run: python -m unittest tools.airsimx.tests.test_cmake_filesystem_probe -v
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).resolve().parent / "fixtures" / "cmake_filesystem_probe"


class CMakeFilesystemProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if os.name != "posix":
            raise unittest.SkipTest("Linux/Unix CMake compiler probe only")
        cls.cmake = shutil.which("cmake")
        cls.compiler = (os.environ.get("AIRSIM_TEST_CXX")
                        or shutil.which("c++")
                        or shutil.which("clang++"))
        if not cls.cmake or not cls.compiler:
            raise unittest.SkipTest("cmake and a C++ compiler are required")

    def _configure_and_build(self, build_dir: Path, extra_args: list[str]) -> str:
        command = [
            self.cmake,
            "-S", str(FIXTURE),
            "-B", str(build_dir),
            f"-DAIRSIM_SOURCE_ROOT={REPO_ROOT}",
            f"-DCMAKE_CXX_COMPILER={self.compiler}",
            "-DCMAKE_BUILD_TYPE=Release",
            *extra_args,
        ]
        compiler_flags = os.environ.get("AIRSIM_TEST_CXX_FLAGS")
        if compiler_flags:
            command.append(f"-DCMAKE_CXX_FLAGS={compiler_flags}")
        standard_libraries = os.environ.get("AIRSIM_TEST_CXX_STANDARD_LIBRARIES")
        if standard_libraries:
            command.append(f"-DCMAKE_CXX_STANDARD_LIBRARIES={standard_libraries}")
        configured = subprocess.run(
            command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(configured.returncode, 0, configured.stdout)

        built = subprocess.run(
            [self.cmake, "--build", str(build_dir), "--target", "directory_iterator_probe"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(built.returncode, 0, built.stdout)

        executable = build_dir / "directory_iterator_probe"
        ran = subprocess.run(
            [str(executable)], cwd=build_dir, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(ran.returncode, 0, ran.stdout)
        selected = (build_dir / "filesystem-extra-library.txt").read_text()
        cache = (build_dir / "CMakeCache.txt").read_text()
        probe = re.search(
            r"^AIRSIM_CXX17_FILESYSTEM_LINKS_WITHOUT_EXTRA_LIB:INTERNAL=(.*)$",
            cache,
            re.MULTILINE,
        )
        self.assertIsNotNone(probe, cache)
        direct_link_available = probe.group(1) in ("1", "TRUE", "ON")
        self.assertEqual(selected, "" if direct_link_available else "c++fs")

        link_files = list(build_dir.rglob("link.txt"))
        self.assertTrue(link_files, "CMake did not generate a link command")
        link_commands = "\n".join(path.read_text() for path in link_files)
        if selected:
            self.assertIn("-lc++fs", link_commands)
        else:
            self.assertNotIn("-lc++fs", link_commands)
        return selected

    def test_uses_the_toolchains_direct_filesystem_link_when_available(self) -> None:
        with tempfile.TemporaryDirectory(prefix="airsim-fs-direct-") as temporary:
            selected = self._configure_and_build(Path(temporary) / "build", [])
            self.assertIn(selected, ("", "c++fs"))

    def test_try_compile_uses_cxx17_when_compiler_default_is_cxx14(self) -> None:
        with tempfile.TemporaryDirectory(prefix="airsim-fs-cxx17-") as temporary:
            self._configure_and_build(
                Path(temporary) / "build",
                ["-DAIRSIM_TEST_DEFAULT_CXX14=ON"],
            )

    def test_uses_cxxfs_fallback_when_direct_link_is_unavailable(self) -> None:
        archiver = shutil.which("ar")
        if not archiver:
            self.skipTest("ar is required to provide the isolated fallback library")

        with tempfile.TemporaryDirectory(prefix="airsim-fs-fallback-") as temporary:
            temporary_path = Path(temporary)
            library_dir = temporary_path / "legacy-lib"
            library_dir.mkdir()
            source = temporary_path / "fallback_marker.cpp"
            object_file = temporary_path / "fallback_marker.o"
            source.write_text("int airsim_cxxfs_fallback_marker() { return 0; }\n")
            subprocess.run(
                [self.compiler, "-c", str(source), "-o", str(object_file)],
                check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            subprocess.run(
                [archiver, "rcs", str(library_dir / "libc++fs.a"), str(object_file)],
                check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            # The synthetic archive only exercises CMake's fallback branch
            # and link ordering on a modern toolchain. It is not an old-libc++
            # ABI/runtime build; that requires an actual legacy SDK.
            linker_flags = os.environ.get("AIRSIM_TEST_EXE_LINKER_FLAGS", "").strip()
            linker_flags = f"{linker_flags} -L{library_dir}".strip()

            selected = self._configure_and_build(
                temporary_path / "build",
                [
                    # Seed the CMake check cache to exercise the legacy
                    # decision branch; the following c++fs probe and final
                    # C++17 executable are still compiled and linked here.
                    "-DAIRSIM_CXX17_FILESYSTEM_LINKS_WITHOUT_EXTRA_LIB:INTERNAL=FALSE",
                    f"-DCMAKE_EXE_LINKER_FLAGS={linker_flags}",
                ],
            )
            self.assertEqual(selected, "c++fs")


if __name__ == "__main__":
    unittest.main()
