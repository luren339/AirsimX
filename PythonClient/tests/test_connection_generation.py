from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import airsim

from rpc_sensor_server import SensorRpcServer, wait_for_idle_disconnect


class ConnectionGenerationTests(unittest.TestCase):
    def test_generation_is_stable_across_calls_on_one_socket(self):
        with SensorRpcServer() as server, airsim.MultirotorClient(
                ip='127.0.0.1', port=server.port, timeout_value=.5) as client:
            self.assertTrue(client.ping())
            generation = getattr(client, 'connection_generation', 0)
            self.assertGreater(generation, 0, 'validated data needs a physical connection identity')
            self.assertEqual(client.listVehicles(), ['UAV_A'])
            self.assertEqual(client.connection_generation, generation)

    def test_idle_reconnection_changes_generation_without_replacing_client(self):
        with SensorRpcServer() as first:
            client = airsim.MultirotorClient(ip='127.0.0.1', port=first.port, timeout_value=.5)
            self.addCleanup(client.close)
            self.assertTrue(client.ping())
            generation = getattr(client, 'connection_generation', 0)
            first.close()
            wait_for_idle_disconnect(client)
            with SensorRpcServer(port=first.port) as second:
                self.assertTrue(client.ping())
                self.assertGreater(getattr(client, 'connection_generation', 0), generation)
                self.assertEqual(second.connections, 1)
