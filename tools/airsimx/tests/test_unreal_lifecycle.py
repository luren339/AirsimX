from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[3]


class UnrealLifecycleTests(unittest.TestCase):
    def test_actor_endplay_overrides_call_the_base(self):
        targets = {
            'AirSimCameraDirector.cpp': 'AAirSimCameraDirector',
            'PIPCamera.cpp': 'APIPCamera',
            'Vehicles/Car/CarPawn.cpp': 'ACarPawn',
            'Vehicles/ComputerVision/ComputerVisionPawn.cpp': 'AComputerVisionPawn',
        }
        for relative, actor in targets.items():
            with self.subTest(actor=actor):
                source = (ROOT / 'Unreal/Plugins/AirSim/Source' / relative).read_text(encoding='utf-8')
                match = re.search(r'void ' + actor + r'::EndPlay\([^\n]*\)\s*\{(.*?)\n\}', source, re.DOTALL)
                self.assertIsNotNone(match)
                calls = re.findall(r'^\s*Super::EndPlay\(EndPlayReason\);\s*$', match.group(1), re.MULTILINE)
                self.assertEqual(len(calls), 1, 'Packaged shutdown must also be tested; this only guards the base call')


if __name__ == '__main__':
    unittest.main()
