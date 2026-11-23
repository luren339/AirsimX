from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[3]
API = ROOT / 'AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp'


class Px4AdapterContractTests(unittest.TestCase):
    def test_connection_reset_preserves_configured_lockstep(self):
        source = API.read_text(encoding='utf-8')
        body = re.search(r'void resetState\(\)\s*\{(.*?)^        \}', source, re.MULTILINE | re.DOTALL).group(1)
        self.assertRegex(body, r'lock_step_enabled_\s*=\s*connection_info_\.lock_step\s*;')
        self.assertRegex(body, r'lock_step_active_\s*=\s*false\s*;')

    def test_movement_commands_restore_offboard_after_hover(self):
        source = API.read_text(encoding='utf-8')
        helper = re.search(r'void ensureOffboardControl\(\)\s*\{(.*?)\n        \}', source, re.DOTALL)
        self.assertIsNotNone(helper)
        self.assertIn('hasOffboardControl()', helper.group(1))
        self.assertIn('requestControl()', helper.group(1))
        for method in ('commandVelocity', 'commandVelocityZ', 'commandPosition',
                       'commandRollPitchYawZ', 'commandRollPitchYawrateZ', 'commandRollPitchYawThrottle',
                       'commandRollPitchYawrateThrottle', 'commandAngleRatesZ', 'commandAngleRatesThrottle'):
            with self.subTest(method=method):
                body = re.search(r'virtual void ' + method + r'\([^)]*\) override\s*\{(.*?)\n        \}',
                                 source, re.DOTALL).group(1)
                self.assertIn('ensureOffboardControl();', body)

    def test_land_passes_yaw_before_global_coordinates(self):
        source = API.read_text(encoding='utf-8')
        body = re.search(r'virtual bool land\(float timeout_sec\) override\s*\{(.*?)^        \}', source,
                         re.MULTILINE | re.DOTALL).group(1)
        call = re.search(r'mav_vehicle_->land\((.*?)\)\.wait', body, re.DOTALL).group(1)
        expected = ('std::numeric_limits<float>::quiet_NaN(),'
                    'current_state_.global_est.pos.lat,current_state_.global_est.pos.lon,'
                    'current_state_.home.global_pos.alt')
        self.assertEqual(re.sub(r'\s+', '', call), expected)
        declaration = (ROOT / 'MavLinkCom/include/MavLinkVehicle.hpp').read_text(encoding='utf-8')
        self.assertIn('land(float yaw, float lat = 0, float lon = 0, float altitude = 0)', declaration)


if __name__ == '__main__':
    unittest.main()
