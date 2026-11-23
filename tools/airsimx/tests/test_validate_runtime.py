"""Offline tests: public-API doubles only; no simulator or network connection.

Run: python -m unittest discover -s tools/airsimx/tests -p test_validate_runtime.py -v
NumPy/OpenCV are the same image dependencies required by the runtime runner.
"""

import contextlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np


RUNNER_PATH = Path(__file__).resolve().parents[1] / "validate_runtime.py"
SPEC = importlib.util.spec_from_file_location("airsimx_validate_runtime", RUNNER_PATH)
v = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = v
SPEC.loader.exec_module(v)


def xyz(x=0, y=0, z=0):
    return NS(x_val=x, y_val=y, z_val=z)


def quat(w=1):
    return NS(x_val=0, y_val=0, z_val=0, w_val=w)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Future:
    def __init__(self, function):
        self.function = function

    def get(self):
        return self.function()


class ImageRequest:
    def __init__(self, camera, image_type, pixels_as_float, compress):
        self.camera_name = camera
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class Server:
    """A shared vehicle behind multiple independently closable client handles."""

    def __init__(self, clock):
        self.clock = clock
        self.epoch = clock()
        self.position = [0.0, 0.0, 0.68]
        self.landed = 0
        self.control = False
        self.armed = False
        self.events = []
        self.clients = []
        self.fail_takeoff = False
        self.landing_stuck_flying = False
        self.position_stuck = False
        self.velocity_stuck = False
        self.state_defect = None
        self.stale_gps = False
        self.image_defect = None
        self.image_batches = 0
        self.empty_images_response = []
        self.assets = ["/Game/FlyingCPP/Meshes/ValidationFixture"]
        rgb = np.zeros((144, 256, 3), dtype=np.uint8)
        rgb[:, :, 0] = np.arange(256, dtype=np.uint8)
        rgb[:, :, 1] = np.arange(144, dtype=np.uint8).reshape(144, 1)
        self.raw = rgb.tobytes()
        self.png = cv2.imencode(".png", rgb)[1].tobytes()

    def timestamp(self):
        return 1 + int((self.clock() - self.epoch) * 1e9)

    def module(self):
        server = self

        def create(**kwargs):
            self.events.append(("connect", kwargs))
            client = Client(server)
            self.clients.append(client)
            return client

        return NS(MultirotorClient=create, LandedState=NS(Landed=0, Flying=1),
                  ImageType=NS(Scene=0, DepthPlanar=1), ImageRequest=ImageRequest)


class Client:
    def __init__(self, server):
        self.server = server
        self.closed = False

    def close(self):
        self.closed = True
        self.server.events.append(("close", id(self)))

    def ping(self):
        return True

    def getClientVersion(self):
        return 1

    def getServerVersion(self):
        return 1

    def getMinRequiredServerVersion(self):
        return 1

    def getMinRequiredClientVersion(self):
        return 1

    def listVehicles(self):
        return ["SimpleFlight"]

    def simIsPause(self):
        return False

    def getSettingsString(self):
        return v.PROFILE_PATH.read_text(encoding="utf-8")

    def isApiControlEnabled(self, vehicle_name):
        return self.server.control

    def enableApiControl(self, enabled, vehicle_name):
        self.server.events.append(("control", enabled))
        self.server.control = enabled

    def armDisarm(self, armed, vehicle_name):
        self.server.events.append(("arm", armed))
        if not armed and self.server.landed != 0:
            raise AssertionError("unsafe midair disarm")
        self.server.armed = armed
        return True

    def getMultirotorState(self, vehicle_name):
        s = self.server
        position = xyz(*s.position)
        orientation = quat()
        timestamp = s.timestamp()
        if s.state_defect == "nan":
            position.x_val = math.nan
        if s.state_defect == "quaternion":
            orientation.w_val = 2
        if s.state_defect == "timestamp":
            timestamp = 1
        return NS(timestamp=timestamp, landed_state=s.landed,
                  kinematics_estimated=NS(position=position, linear_velocity=xyz(), angular_velocity=xyz(), orientation=orientation))

    def takeoffAsync(self, timeout_sec, vehicle_name):
        assert timeout_sec == 20
        def action():
            self.server.events.append(("takeoff", timeout_sec))
            self.server.position[2] = -1.5
            self.server.landed = 1
            self.server.clock.sleep(2)
            if self.server.fail_takeoff:
                raise TimeoutError("simulated lost takeoff response while airborne")
            return True
        return Future(action)

    def hoverAsync(self, vehicle_name):
        return Future(lambda: True)

    def landAsync(self, timeout_sec, vehicle_name):
        assert timeout_sec == 30
        def action():
            self.server.events.append(("land", timeout_sec))
            self.server.position[2] = 0.68
            if not self.server.landing_stuck_flying:
                self.server.landed = 0
            self.server.clock.sleep(2)
            return True
        return Future(action)

    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec, vehicle_name):
        assert timeout_sec == 20
        assert velocity == 1
        def action():
            self.server.events.append(("position", [x, y, z]))
            if not self.server.position_stuck:
                self.server.position = [x, y, z]
            self.server.clock.sleep(2)
            return True
        return Future(action)

    def moveByVelocityAsync(self, vx, vy, vz, duration, vehicle_name):
        assert duration == 2
        def action():
            self.server.events.append(("velocity", [vx, vy, vz, duration]))
            if not self.server.velocity_stuck:
                self.server.position = [p + speed * duration for p, speed in zip(self.server.position, [vx, vy, vz])]
            self.server.clock.sleep(duration)
            return True
        return Future(action)

    def cancelLastTask(self, vehicle_name):
        self.server.events.append(("cancel", None))

    def reset(self):
        self.server.events.append(("reset", None))
        if self.server.armed or self.server.control or self.server.landed != 0:
            raise AssertionError("reset before safe disarm/release")
        self.server.position = [0.0, 0.0, 0.68]
        self.server.epoch = self.server.clock()

    def getHomeGeoPoint(self, vehicle_name):
        return NS(latitude=47.641468, longitude=-122.140165, altitude=122)

    def getImuData(self, imu_name, vehicle_name):
        assert imu_name == "Imu"
        return NS(time_stamp=self.server.timestamp(), linear_acceleration=xyz(0, 0, -9.81),
                  angular_velocity=xyz(), orientation=quat())

    def getGpsData(self, gps_name, vehicle_name):
        assert gps_name == "Gps"
        return NS(time_stamp=1 if self.server.stale_gps else self.server.timestamp(), is_valid=True,
                  gnss=NS(geo_point=self.getHomeGeoPoint(vehicle_name), velocity=xyz(), eph=0.1, epv=0.1, fix_type=3))

    def getBarometerData(self, barometer_name, vehicle_name):
        assert barometer_name == "Barometer"
        return NS(time_stamp=self.server.timestamp(), pressure=99875, altitude=122, qnh=101325)

    def simListAssets(self):
        self.server.events.append(("assets", None))
        return self.server.assets

    def simGetImages(self, requests, vehicle_name):
        s = self.server
        if not requests:
            s.events.append(("empty_images", None))
            return s.empty_images_response
        s.image_batches += 1
        result = []
        for i, req in enumerate(requests):
            response = NS(width=256, height=144, image_type=req.image_type,
                          pixels_as_float=req.pixels_as_float, compress=req.compress,
                          time_stamp=s.timestamp(), message="", image_data_float=[], image_data_uint8=b"")
            if i == 0:
                response.image_data_uint8 = s.png
                if s.image_defect == "png":
                    response.image_data_uint8 = b"not PNG"
                if s.image_defect == "undecodable":
                    response.image_data_uint8 = b"\x89PNG\r\n\x1a\nnot an image"
                if s.image_defect == "blank":
                    response.image_data_uint8 = cv2.imencode(".png", np.zeros((144, 256, 3), dtype=np.uint8))[1].tobytes()
                if s.image_defect == "dimensions":
                    response.width = 255
                if s.image_defect == "descriptor":
                    response.compress = False
                if s.image_defect == "server_error":
                    response.message = "camera unavailable"
            elif i == 1:
                response.image_data_float = [1.0] * (256 * 144)
                if s.image_defect in ("depth_nan", "depth_zero", "depth_negative"):
                    response.image_data_float[0] = {"depth_nan": math.nan, "depth_zero": 0, "depth_negative": -1}[s.image_defect]
                if s.image_defect == "depth_size":
                    response.image_data_float.pop()
            else:
                response.image_data_uint8 = s.raw
                if s.image_defect == "raw_size":
                    response.image_data_uint8 = s.raw[:-1]
                if s.image_defect == "stale_timestamp":
                    response.time_stamp = 1
            result.append(response)
        if s.image_defect == "response_count":
            result.pop()
        return result


def hung_worker(options, main_deadline, final_deadline, cleanup_only=False):
    """Real child for watchdog tests; simulates a client that ignores its timeout."""
    report = v.read_report(options["output"])
    if cleanup_only:
        report["phase"] = "cleanup"
        report["cleanup"] = {"status": "passed", "steps": [{"name": "test_recovery", "result": True}]}
        report["safety"] = dict.fromkeys(report["safety"], False)
        v.write_report(options["output"], report)
        return
    report["phase"] = "acceptance_hover"
    report["checks"].append({"name": "partial evidence before hung RPC", "passed": True})
    report["safety"] = dict.fromkeys(report["safety"], True)
    report["active_operation"] = {"name": "deliberately_hung_call", "deadline_monotonic": time.monotonic() + 0.25}
    v.write_report(options["output"], report)
    time.sleep(30)


def crash_worker(options, main_deadline, final_deadline, cleanup_only=False):
    os._exit(7)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "evidence"
        self.options = dict(host="127.0.0.1", port=41451, vehicle="SimpleFlight", camera="0",
                            output=str(self.output), hover_seconds=60.0, cycles=20, image_batches=10,
                            server_pid=None, rpc_timeout=40.0, max_seconds=1200.0)

    def make_runner(self, **changes):
        self.options.update(changes)
        self.output.mkdir()
        clock = Clock()
        server = Server(clock)
        report = v.initial_report(self.options, clock())
        runner = v.Runner(self.options, report, server.module(), np, cv2, 1100, 1190, clock, clock.sleep)
        runner.save()
        return runner, server

    def test_atomic_report_retries_transient_windows_sharing_violation(self):
        self.output.mkdir()
        original = os.replace
        attempts = []
        def sharing_violation_once(source, target):
            attempts.append((source, target))
            if len(attempts) == 1:
                raise PermissionError("sharing violation")
            return original(source, target)
        with mock.patch.object(v.os, "replace", side_effect=sharing_violation_once), mock.patch.object(v.time, "sleep"):
            v.write_report(self.output, {"value": math.nan})
        self.assertEqual(len(attempts), 2)
        self.assertEqual(v.read_report(self.output), {"value": "nan"})
        with mock.patch.object(v.os, "replace", side_effect=PermissionError("persistent")), mock.patch.object(v.time, "sleep"), self.assertRaises(PermissionError):
            v.write_report(self.output, {"value": 2})
        self.assertEqual(v.read_report(self.output), {"value": "nan"})

    def test_default_profile_and_loopback_settings(self):
        options = v.parse_args(["--output", str(self.output)])
        self.assertEqual((options["hover_seconds"], options["cycles"], options["image_batches"]), (60, 20, 10))
        profile = v.initial_report(options, 0)["profile"]
        self.assertTrue(profile["full_acceptance_counts"])
        self.assertEqual(profile["settings"]["LocalHostIp"], "127.0.0.1")
        self.assertEqual(profile["settings"]["Vehicles"]["SimpleFlight"]["DefaultVehicleState"], "Disarmed")
        self.assertEqual(profile["thresholds"]["hover_drift_m_lte"], 1)
        self.assertEqual(profile["thresholds"]["depth_finite_positive_fraction"], 1)
        self.assertEqual(len(profile["settings_sha256"]), 64)

    def test_invalid_cli_values_rejected(self):
        for extra in (["--hover-seconds", "nan"], ["--hover-seconds", "inf"], ["--hover-seconds", "0"],
                      ["--cycles", "0"], ["--image-batches", "0"], ["--rpc-timeout", "1"],
                      ["--rpc-timeout", "nan"], ["--max-seconds", "inf"], ["--max-seconds", "100"],
                      ["--port", "0"], ["--server-pid", "-1"], ["--vehicle", ""]):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                v.parse_args(["--output", str(self.output)] + extra)

    def test_existing_output_never_reused(self):
        self.output.mkdir()
        marker = self.output / "result.json"
        marker.write_text("keep", encoding="utf-8")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            v.parse_args(["--output", str(self.output)])
        with self.assertRaises(FileExistsError):
            v.supervise(self.options)
        self.assertEqual(marker.read_text(), "keep")

    def test_shortened_counts_not_acceptance(self):
        for key, value in (("hover_seconds", 59), ("cycles", 19), ("image_batches", 9)):
            with self.subTest(key=key):
                report = v.initial_report(dict(self.options, **{key: value}), 0)
                self.assertFalse(report["profile"]["full_acceptance_counts"])
                self.assertEqual(report["profile"]["name"], "custom-shortened-smoke")

    def test_close_prefers_public_and_falls_back_only_when_missing(self):
        public, legacy = mock.Mock(), mock.Mock()
        v.close_client(NS(close=public, client=NS(close=legacy)))
        public.assert_called_once_with()
        legacy.assert_not_called()
        v.close_client(NS(client=NS(close=legacy)))
        legacy.assert_called_once_with()
        with self.assertRaises(v.ValidationFailure):
            v.close_client(NS())
        public.side_effect = RuntimeError("close failure")
        with self.assertRaises(RuntimeError):
            v.close_client(NS(close=public, client=NS(close=legacy)))
        self.assertEqual(legacy.call_count, 1)

    def test_full_profile_with_fake_server_and_real_image_decoders(self):
        runner, server = self.make_runner()
        with contextlib.redirect_stdout(io.StringIO()):
            runner.run()
            runner.cleanup()
        report = v.read_report(self.output)
        self.assertEqual(report["errors"], [])
        self.assertTrue(all(c["passed"] for c in report["checks"]))
        self.assertEqual(report["cleanup"]["status"], "not_needed")
        self.assertFalse(any(report["safety"].values()))
        self.assertEqual(len(report["cycles"]), 20)
        self.assertTrue(all(c["completed"] for c in report["cycles"]))
        self.assertEqual(len(report["images"]), 10)
        self.assertTrue(all(i["completed"] for i in report["images"]))
        self.assertEqual(server.image_batches, 10)
        self.assertEqual(sum(e[0] == "empty_images" for e in server.events), 1)
        self.assertEqual(sum(e[0] == "assets" for e in server.events), 1)
        self.assertEqual(report["asset_query"]["count"], 1)
        operations = [op["name"] for op in report["operations"]]
        self.assertLess(operations.index("simGetImages(empty)"), operations.index("simListAssets"))
        self.assertLess(operations.index("simListAssets"), operations.index("simGetImages(mixed)"))
        self.assertEqual(len(report["missions"]), 3)
        self.assertTrue(all(m["completed"] for m in report["missions"]))
        self.assertEqual(len(report["sensors"]), 3)
        self.assertGreaterEqual(report["hover"][0]["elapsed_seconds"], 60)
        self.assertEqual([r["after_cycle"] for r in report["reconnects"]], [5, 10, 15, 20])
        self.assertEqual(len(server.clients), 5)
        self.assertTrue(all(client.closed for client in server.clients))
        self.assertEqual(sum(e[0] == "reset" for e in server.events), 20)
        self.assertEqual(sum(e[0] == "takeoff" for e in server.events), 21)
        self.assertEqual(np.load(self.output / "depth.npy", allow_pickle=False).shape, (144, 256))
        self.assertEqual(np.load(self.output / "scene-raw.npy", allow_pickle=False).shape, (144, 256, 3))
        self.assertIsNotNone(cv2.imread(str(self.output / "scene.png")))
        # Strictly increasing is required between spaced samples, not across reset.
        old = next(s for s in reversed(report["states"]) if s["epoch"] == "initial")
        reset = next(s for s in report["states"] if s["epoch"] == "reset_01")
        self.assertLess(reset["timestamp"], old["timestamp"])

    def test_airborne_failure_cleanup_lands_before_disarm(self):
        runner, server = self.make_runner(hover_seconds=1, cycles=1, image_batches=1)
        server.fail_takeoff = True
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(TimeoutError):
                runner.run()
            runner.cleanup()
        events = server.events
        land = next(i for i, e in enumerate(events) if e[0] == "land")
        disarm = next(i for i, e in enumerate(events) if e == ("arm", False))
        release = next(i for i, e in enumerate(events) if e == ("control", False))
        self.assertLess(land, disarm)
        self.assertLess(disarm, release)
        self.assertEqual(runner.report["cleanup"]["status"], "passed")
        self.assertFalse(any(runner.report["safety"].values()))
        self.assertTrue(all(c.closed for c in server.clients))
        self.assertEqual(len(server.clients), 2)

    def test_landed_state_failure_never_disarms_or_releases_in_air(self):
        runner, server = self.make_runner(hover_seconds=1, cycles=1, image_batches=1)
        server.landing_stuck_flying = True
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(v.ValidationFailure, "landed state"):
                runner.run()
            runner.cleanup()
        self.assertNotIn(("arm", False), server.events)
        self.assertNotIn(("control", False), server.events)
        self.assertEqual(runner.report["cleanup"]["status"], "failed")
        self.assertTrue(runner.report["safety"]["may_be_airborne"])
        self.assertTrue(all(c.closed for c in server.clients))

    def test_foreign_control_not_taken_over(self):
        runner, server = self.make_runner()
        server.control = True
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(v.ValidationFailure, "initial API control"):
                runner.run()
            runner.cleanup()
        self.assertFalse(any(e[0] in ("arm", "land", "cancel", "control") for e in server.events))
        self.assertTrue(server.control)

    def test_nonfinite_and_bad_quaternion_fail_with_json_evidence(self):
        for defect, expected in (("nan", "finite state"), ("quaternion", "quaternion normalized"), ("timestamp", "clock advances")):
            with self.subTest(defect=defect):
                self.output = Path(self.temp.name) / defect
                runner, server = self.make_runner(output=str(self.output))
                server.state_defect = defect
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, expected):
                    runner.run()
                report = v.read_report(self.output)
                self.assertFalse(report["checks"][-1]["passed"])
                if defect == "nan":
                    self.assertEqual(report["states"][-1]["position"][0], "nan")

    def test_each_sensor_clock_must_advance(self):
        runner, server = self.make_runner()
        runner.client = server.module().MultirotorClient()
        server.stale_gps = True
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, "all sensor timestamps advance"):
            runner.sensors()
        self.assertEqual(len(v.read_report(self.output)["sensors"]), 2)

    def test_empty_image_response_must_be_an_empty_list(self):
        runner, server = self.make_runner(image_batches=1)
        runner.client = server.module().MultirotorClient()
        server.empty_images_response = ["unexpected response"]
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, "empty image request"):
            runner.images()
        report = v.read_report(self.output)
        self.assertFalse(report["checks"][-1]["passed"])
        self.assertEqual(report["checks"][-1]["detail"]["response_count"], 1)
        self.assertEqual(server.image_batches, 0)
        self.assertNotIn(("assets", None), server.events)

    def test_empty_or_nonlist_asset_result_fails_before_mixed_batches(self):
        for assets in ([], None, "not a list"):
            with self.subTest(assets=assets):
                self.output = Path(self.temp.name) / ("assets_%s" % type(assets).__name__)
                runner, server = self.make_runner(output=str(self.output), image_batches=1)
                runner.client = server.module().MultirotorClient()
                server.assets = assets
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, "nonempty asset list"):
                    runner.images()
                report = v.read_report(self.output)
                self.assertFalse(report["checks"][-1]["passed"])
                self.assertEqual(report["asset_query"]["count"], 0 if isinstance(assets, list) else None)
                self.assertEqual(server.image_batches, 0)
                self.assertEqual(report["images"], [])

    def test_malformed_images_fail_instead_of_becoming_passes(self):
        defects = {"png": "PNG signature", "undecodable": "PNG decoding", "blank": "PNG spatial variation",
                   "dimensions": "dimensions", "descriptor": "request/response", "server_error": "no server error",
                   "raw_size": "raw Scene byte count", "depth_size": "depth element count",
                   "depth_nan": "depth entirely", "depth_zero": "depth entirely", "depth_negative": "depth entirely",
                   "response_count": "three mixed", "stale_timestamp": "all image timestamps advance"}
        for defect, expected in defects.items():
            with self.subTest(defect=defect):
                self.output = Path(self.temp.name) / defect
                runner, server = self.make_runner(output=str(self.output), image_batches=2)
                runner.client = server.module().MultirotorClient()
                server.image_defect = defect
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, expected):
                    runner.images()
                report = v.read_report(self.output)
                self.assertFalse(report["checks"][-1]["passed"])
                self.assertFalse(report["images"][-1]["completed"])

    def test_true_position_command_without_motion_is_failure(self):
        runner, server = self.make_runner(hover_seconds=1, cycles=1, image_batches=1)
        server.position_stuck = True
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, "position reached"):
            runner.run()

    def test_true_velocity_command_without_motion_is_failure(self):
        runner, server = self.make_runner(hover_seconds=1, cycles=1, image_batches=1)
        server.velocity_stuck = True
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(v.ValidationFailure, "makes forward progress"):
            runner.run()

    def test_operation_and_wait_deadlines_leave_partial_evidence(self):
        runner, _ = self.make_runner()
        with self.assertRaises(TimeoutError):
            runner.call("slow", lambda: runner.clock.sleep(41))
        report = v.read_report(self.output)
        self.assertIsNone(report["active_operation"])
        self.assertEqual(report["operations"][-1]["outcome"], "failed")
        self.assertEqual(report["operations"][-1]["timeout_seconds"], 40)
        runner.deadline = runner.clock() + 0.5
        with self.assertRaises(TimeoutError):
            runner.wait(1)

    def test_watchdog_distinguishes_rpc_main_and_cleanup_deadlines(self):
        report = {"phase": "flight", "active_operation": {"name": "land", "deadline_monotonic": 10}}
        self.assertIn("operation", v.watchdog_reason(report, 11, 50, 60))
        report["active_operation"] = None
        self.assertIn("main", v.watchdog_reason(report, 51, 50, 60))
        report["phase"] = "cleanup"
        self.assertIsNone(v.watchdog_reason(report, 51, 50, 60))
        self.assertIn("cleanup", v.watchdog_reason(report, 61, 50, 60))

    def test_real_subprocess_watchdog_preserves_evidence_and_runs_recovery(self):
        options = dict(self.options, max_seconds=6, hover_seconds=1, cycles=1, image_batches=1)
        started = time.monotonic()
        with mock.patch.object(v, "CLEANUP_SECONDS", 1), mock.patch.object(v, "STOP_SECONDS", 0.2), contextlib.redirect_stdout(io.StringIO()):
            code = v.supervise(options, worker_target=hung_worker)
        self.assertEqual(code, 1)
        self.assertLess(time.monotonic() - started, 6)
        report = v.read_report(self.output)
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["acceptance_passed"])
        self.assertEqual(report["checks"][0]["name"], "partial evidence before hung RPC")
        self.assertTrue(any("deliberately_hung_call" in e["message"] for e in report["errors"]))
        self.assertEqual(report["cleanup"]["status"], "passed")
        self.assertFalse(any(report["safety"].values()))

    def test_child_crash_is_failure_not_pass(self):
        options = dict(self.options, max_seconds=6)
        with mock.patch.object(v, "CLEANUP_SECONDS", 1), mock.patch.object(v, "STOP_SECONDS", 0.2), contextlib.redirect_stdout(io.StringIO()):
            code = v.supervise(options, worker_target=crash_worker)
        self.assertEqual(code, 1)
        report = v.read_report(self.output)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(any("code 7" in e["message"] for e in report["errors"]))

    def test_import_failure_gets_incremental_json_and_log(self):
        self.output.mkdir()
        report = v.initial_report(self.options, time.monotonic())
        v.write_report(self.output, report)
        with mock.patch.object(v, "load_runtime", side_effect=ImportError("dependency missing")):
            v.worker_main(self.options, time.monotonic() + 10, time.monotonic() + 20)
        saved = v.read_report(self.output)
        self.assertEqual(saved["worker_outcome"], "failed")
        self.assertEqual(saved["cleanup"]["status"], "not_needed")
        self.assertIn("dependency missing", saved["errors"][0]["message"])
        self.assertIn("dependency missing", (self.output / "worker.log").read_text(encoding="utf-8"))

    def test_optional_pid_metrics_failure_is_diagnostic(self):
        runner, _ = self.make_runner(server_pid=99999999)
        with mock.patch.dict(sys.modules, {"psutil": NS(Process=mock.Mock(side_effect=PermissionError("unavailable")))}):
            runner.metrics("offline")
        self.assertEqual(runner.report["errors"], [])
        self.assertTrue(runner.report["metrics"][0]["diagnostic_only"])
        self.assertIn("unavailable", runner.report["metrics"][0])
        self.assertTrue(runner.report["warnings"])


if __name__ == "__main__":
    unittest.main()
