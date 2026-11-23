#!/usr/bin/env python3
"""Bounded, destructive-in-simulation AirsimX runtime acceptance check.

Start a dedicated Blocks simulator with settings/runtime-validation.json first.
This script never starts/stops Unreal, changes global settings, or kills a server.
It imports the repository's public PythonClient/airsim package, not a private RPC
implementation. See --help. Shortened counts are smoke tests, not v0.1 acceptance.
"""

import argparse
import contextlib
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import sys
import time
import traceback
from datetime import datetime, timezone


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = Path(__file__).resolve().parent / "settings" / "runtime-validation.json"
# Flight/image tolerances retain the earlier Blocks checks. The short mission
# bounds are explicit API sanity checks, not flight-model/physical-realism claims.
THRESHOLDS = {
    "takeoff_height_m_gt": 1.0,
    "hover_drift_m_lte": 1.0,
    # M1 asks for both position and attitude error over the 60 s hover. Ten
    # degrees is a sanity bound for a stationary hover, not a control-quality
    # claim; the recorded value matters more than the threshold.
    "hover_attitude_error_deg_lte": 10.0,
    "quaternion_norm_squared_error_lt": 0.02,
    "land_height_error_m_lt": 0.5,
    "land_speed_m_s_lt": 0.3,
    "reset_position_error_m_lt": 0.5,
    "imu_acceleration_m_s2_gt": 8.5,
    "imu_acceleration_m_s2_lt": 11.5,
    "gps_home_lat_lon_error_degrees_lt": 0.01,
    "barometer_pressure_pa_gt": 80000.0,
    "barometer_pressure_pa_lt": 110000.0,
    "image_width": 256,
    "image_height": 144,
    "png_unique_colors_gt": 16,
    "depth_finite_positive_fraction": 1.0,
    "state_sample_interval_seconds": 0.5,
    "sensor_samples": 3,
    "sensor_interval_seconds": 1.5,
    "image_interval_seconds": 0.2,
    "cycle_hover_seconds": 1.0,
    "reset_settle_seconds": 1.5,
    "land_settle_timeout_seconds": 5.0,
    "takeoff_timeout_seconds": 20.0,
    "land_timeout_seconds": 30.0,
    "reconnect_every_cycles": 5,
    "position_mission_offset_m": 2.0,
    "position_mission_speed_m_s": 1.0,
    "position_mission_timeout_seconds": 20.0,
    "position_mission_error_m_lt": 0.5,
    "velocity_mission_speed_m_s": 0.5,
    "velocity_mission_seconds": 2.0,
    "velocity_mission_progress_m_gt": 0.25,
    "velocity_mission_displacement_m_lt": 2.0,
    "velocity_mission_altitude_error_m_lt": 0.5,
}
CLEANUP_SECONDS = 90.0
STOP_SECONDS = 3.0


class ValidationFailure(RuntimeError):
    pass


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def json_safe(value):
    """Preserve invalid numerical evidence without emitting non-standard JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def write_report(output, report):
    path = Path(output) / "result.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    # Windows readers may briefly hold a handle without FILE_SHARE_DELETE.
    # Retry that sharing violation only, with a finite half-second budget.
    for attempt in range(51):
        try:
            os.replace(str(temporary), str(path))
            return
        except PermissionError:
            if attempt == 50:
                raise
            time.sleep(0.01)


def read_report(output):
    for attempt in range(51):
        try:
            return json.loads((Path(output) / "result.json").read_text(encoding="utf-8"))
        except PermissionError:
            if attempt == 50:
                raise
            time.sleep(0.01)


def add_error(report, where, error):
    report["errors"].append({"where": where, "type": type(error).__name__, "message": str(error), "time": utc_now()})


def vector(value):
    return [float(value.x_val), float(value.y_val), float(value.z_val)]


def quaternion(value):
    return vector(value) + [float(value.w_val)]


def norm(values):
    return math.sqrt(sum(x * x for x in values))


def distance(a, b):
    return norm([x - y for x, y in zip(a, b)])


def quaternion_angle_deg(a, b):
    """Rotation angle between two quaternions stored as [x, y, z, w].

    The w component sits last here because ``quaternion()`` builds the list
    from AirSim's x/y/z fields and appends w.
    """
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def close_client(client):
    """Prefer public close(); retain compatibility with the original SDK."""
    close = getattr(client, "close", None)
    if not callable(close):
        close = getattr(getattr(client, "client", None), "close", None)
    if not callable(close):
        raise ValidationFailure("Python client exposes neither close() nor client.close()")
    close()


def load_runtime():
    sys.path.insert(0, str(ROOT / "PythonClient"))
    import airsim
    import cv2
    import numpy
    if Path(airsim.__file__).resolve().parent != ROOT / "PythonClient" / "airsim":
        raise ValidationFailure("airsim was not imported from this repository's PythonClient")
    return airsim, numpy, cv2


def initial_report(options, started):
    profile_bytes = PROFILE_PATH.read_bytes()
    full = options["hover_seconds"] >= 60 and options["cycles"] >= 20 and options["image_batches"] >= 10
    return {
        "schema_version": 1,
        "status": "running",
        "acceptance_passed": False,
        "started_utc": utc_now(),
        "started_monotonic": started,
        "profile": {
            "name": "airsimx-v0.1-runtime" if full else "custom-shortened-smoke",
            "full_acceptance_counts": full,
            "minimum_counts": {"hover_seconds": 60, "cycles": 20, "image_batches": 10},
            "options": options,
            "thresholds": dict(THRESHOLDS),
            "settings_file": PROFILE_PATH.relative_to(ROOT).as_posix(),
            "settings_sha256": hashlib.sha256(profile_bytes).hexdigest(),
            "settings": json.loads(profile_bytes),
            "cleanup_reserve_seconds": CLEANUP_SECONDS,
            "scope": "Blocks/SimpleFlight API and numerical sanity; no physical-realism, leak, RHI or packaged-build certification",
        },
        "environment": {"python": sys.version, "platform": platform.platform()},
        "phase": "starting",
        "active_operation": None,
        "operations": [],
        "checks": [],
        "states": [],
        "hover": [],
        "missions": [],
        "cycles": [],
        "sensors": [],
        "images": [],
        "reconnects": [],
        "metrics": [],
        "warnings": [],
        "errors": [],
        "safety": {"control_may_be_owned": False, "may_be_armed": False, "may_be_airborne": False},
        "cleanup": {"status": "not_started", "steps": []},
        "worker_outcome": None,
    }


class Runner:
    """Single-threaded client owner; supervisor enforces RPC and wall deadlines."""

    def __init__(self, options, report, airsim, numpy, cv2, main_deadline, final_deadline,
                 clock=time.monotonic, sleep=time.sleep):
        self.options, self.report = options, report
        self.airsim, self.np, self.cv2 = airsim, numpy, cv2
        self.main_deadline, self.final_deadline = main_deadline, final_deadline
        self.clock, self.sleep = clock, sleep
        self.deadline = main_deadline
        self.client = None
        self.epoch = "initial"
        self.last_state_timestamp = None
        self.server_identity = None

    def save(self):
        write_report(self.options["output"], self.report)

    def phase(self, name):
        self.report["phase"] = name
        self.save()
        print(name, flush=True)

    def check(self, name, condition, detail=None):
        self.report["checks"].append({"name": name, "passed": bool(condition), "detail": detail})
        self.save()
        if not condition:
            raise ValidationFailure(name)

    def remaining(self):
        remaining = self.deadline - self.clock()
        if remaining <= 0:
            raise TimeoutError("validation wall-clock deadline reached")
        return remaining

    def call(self, name, function):
        started = self.clock()
        budget = min(self.options["rpc_timeout"], self.remaining())
        operation = {"name": name, "phase": self.report["phase"], "started_monotonic": started,
                     "deadline_monotonic": started + budget, "timeout_seconds": budget, "outcome": "running"}
        self.report["active_operation"] = operation
        self.report["operations"].append(operation)
        self.save()  # Durable before side effects or a potentially hung RPC.
        try:
            result = function()
            if self.clock() > operation["deadline_monotonic"]:
                raise TimeoutError("operation exceeded its deadline: " + name)
            operation["outcome"] = "returned"
            return result
        except BaseException as error:
            operation["outcome"] = "failed"
            operation["error"] = repr(error)
            raise
        finally:
            operation["elapsed_seconds"] = self.clock() - started
            self.report["active_operation"] = None
            self.save()

    def wait(self, seconds):
        remaining = self.remaining()
        if seconds > remaining:
            raise TimeoutError("insufficient time for required sample interval")
        self.sleep(seconds)

    def safety(self, **values):
        self.report["safety"].update(values)
        self.save()

    def new_client(self):
        return self.call("create_client", lambda: self.airsim.MultirotorClient(
            ip=self.options["host"], port=self.options["port"], timeout_value=self.options["rpc_timeout"]))

    def connect(self):
        self.client = self.new_client()
        self.check("ping", self.call("ping", self.client.ping) is True)
        versions = {
            "client": self.client.getClientVersion(),
            "server": self.call("getServerVersion", self.client.getServerVersion),
            "minimum_server": self.client.getMinRequiredServerVersion(),
            "minimum_client": self.call("getMinRequiredClientVersion", self.client.getMinRequiredClientVersion),
        }
        self.check("compatible client/server versions", versions["server"] >= versions["minimum_server"]
                   and versions["client"] >= versions["minimum_client"], versions)
        vehicles = self.call("listVehicles", self.client.listVehicles)
        self.check("selected vehicle exists", self.options["vehicle"] in vehicles, vehicles)
        self.check("simulation not paused", self.call("simIsPause", self.client.simIsPause) is False)

    def state(self, label):
        state = self.call("getMultirotorState", lambda: self.client.getMultirotorState(vehicle_name=self.options["vehicle"]))
        k = state.kinematics_estimated
        sample = {"label": label, "epoch": self.epoch, "wall_monotonic": self.clock(),
                  "timestamp": int(state.timestamp), "landed_state": int(state.landed_state),
                  "position": vector(k.position), "linear_velocity": vector(k.linear_velocity),
                  "angular_velocity": vector(k.angular_velocity), "quaternion": quaternion(k.orientation)}
        self.report["states"].append(sample)
        numbers = sample["position"] + sample["linear_velocity"] + sample["angular_velocity"] + sample["quaternion"]
        self.check(label + " finite state", all(math.isfinite(x) for x in numbers))
        q_error = abs(sum(x * x for x in sample["quaternion"]) - 1.0)
        self.check(label + " quaternion normalized", q_error < THRESHOLDS["quaternion_norm_squared_error_lt"], q_error)
        self.check(label + " positive timestamp", sample["timestamp"] > 0, sample["timestamp"])
        if self.last_state_timestamp is not None:
            self.check(label + " timestamp nondecreasing in reset epoch", sample["timestamp"] >= self.last_state_timestamp)
        self.last_state_timestamp = sample["timestamp"]
        return sample

    def grounded(self, label, ground=None):
        until = min(self.deadline, self.clock() + THRESHOLDS["land_settle_timeout_seconds"])
        while True:
            sample = self.state(label)
            is_landed = sample["landed_state"] == self.airsim.LandedState.Landed
            is_still = norm(sample["linear_velocity"]) < THRESHOLDS["land_speed_m_s_lt"]
            height_error = 0.0 if ground is None else abs(sample["position"][2] - ground[2])
            if is_landed and is_still and height_error < THRESHOLDS["land_height_error_m_lt"]:
                break
            if self.clock() >= until:
                break
            self.wait(min(0.2, until - self.clock()))
        self.check(label + " landed state", is_landed, sample["landed_state"])
        self.check(label + " stationary", is_still, norm(sample["linear_velocity"]))
        self.check(label + " near measured ground height", height_error < THRESHOLDS["land_height_error_m_lt"], height_error)
        return sample

    def release(self):
        vehicle = self.options["vehicle"]
        self.check("disarm command", self.call("armDisarm(False)", lambda: self.client.armDisarm(False, vehicle_name=vehicle)) is True)
        self.safety(may_be_armed=False, may_be_airborne=False)
        self.call("enableApiControl(False)", lambda: self.client.enableApiControl(False, vehicle_name=vehicle))
        self.check("API control released", self.call("isApiControlEnabled", lambda: self.client.isApiControlEnabled(vehicle_name=vehicle)) is False)
        self.safety(control_may_be_owned=False)

    def flight(self, label, ground, hover_seconds, include_missions=False):
        vehicle = self.options["vehicle"]
        self.safety(control_may_be_owned=True)
        self.call("enableApiControl(True)", lambda: self.client.enableApiControl(True, vehicle_name=vehicle))
        self.check("API control enabled", self.call("isApiControlEnabled", lambda: self.client.isApiControlEnabled(vehicle_name=vehicle)) is True)
        self.safety(may_be_armed=True)
        self.check("arm command", self.call("armDisarm(True)", lambda: self.client.armDisarm(True, vehicle_name=vehicle)) is True)
        self.safety(may_be_airborne=True)
        self.check("takeoff command", self.call("takeoffAsync.get", lambda: self.client.takeoffAsync(
            timeout_sec=THRESHOLDS["takeoff_timeout_seconds"], vehicle_name=vehicle).get()) is True)
        airborne = self.state(label + " takeoff")
        gain = ground[2] - airborne["position"][2]
        self.check(label + " takeoff height", gain > THRESHOLDS["takeoff_height_m_gt"], gain)
        self.check(label + " flying state", airborne["landed_state"] == self.airsim.LandedState.Flying)
        self.check("hover command", self.call("hoverAsync.get", lambda: self.client.hoverAsync(vehicle_name=vehicle).get()) is True)
        anchor = self.state(label + " hover start")
        started, last_timestamp, max_drift = self.clock(), anchor["timestamp"], 0.0
        max_attitude_deg, attitude_sum_deg = 0.0, 0.0
        hover = {"label": label, "requested_seconds": hover_seconds, "elapsed_seconds": 0.0,
                 "max_drift_m": 0.0, "max_attitude_error_deg": 0.0,
                 "mean_attitude_error_deg": 0.0, "samples": 0, "completed": False}
        self.report["hover"].append(hover)
        self.save()
        while self.clock() - started < hover_seconds:
            self.wait(min(THRESHOLDS["state_sample_interval_seconds"], hover_seconds - (self.clock() - started)))
            sample = self.state(label + " hover")
            max_drift = max(max_drift, distance(anchor["position"], sample["position"]))
            attitude_deg = quaternion_angle_deg(anchor["quaternion"], sample["quaternion"])
            max_attitude_deg = max(max_attitude_deg, attitude_deg)
            attitude_sum_deg += attitude_deg
            samples = hover["samples"] + 1
            hover.update(elapsed_seconds=self.clock() - started, max_drift_m=max_drift,
                         max_attitude_error_deg=max_attitude_deg,
                         mean_attitude_error_deg=attitude_sum_deg / samples, samples=samples)
            self.check(label + " hover timestamp advances", sample["timestamp"] > last_timestamp)
            self.check(label + " remains flying", sample["landed_state"] == self.airsim.LandedState.Flying)
            self.check(label + " hover drift", max_drift <= THRESHOLDS["hover_drift_m_lte"], max_drift)
            self.check(label + " hover attitude error",
                       max_attitude_deg <= THRESHOLDS["hover_attitude_error_deg_lte"], max_attitude_deg)
            last_timestamp = sample["timestamp"]
        hover["completed"] = True
        self.save()
        if include_missions:
            self.missions(anchor["position"])
        self.check("land command", self.call("landAsync.get", lambda: self.client.landAsync(
            timeout_sec=THRESHOLDS["land_timeout_seconds"], vehicle_name=vehicle).get()) is True)
        landed = self.grounded(label + " landing", ground)
        self.release()
        return {"takeoff_height_m": gain, "hover_max_drift_m": max_drift,
                "hover_max_attitude_error_deg": max_attitude_deg,
                "landed_state": landed["landed_state"],
                "land_height_error_m": abs(landed["position"][2] - ground[2]), "land_speed_m_s": norm(landed["linear_velocity"])}

    def missions(self, origin):
        """Small, bounded NED commands, returning above the measured landing site."""
        self.phase("position_velocity_missions")
        vehicle = self.options["vehicle"]

        def position_mission(name, target):
            row = {"name": name, "target_ned_m": target, "completed": False}
            self.report["missions"].append(row)
            self.save()
            result = self.call("moveToPositionAsync.get", lambda: self.client.moveToPositionAsync(
                target[0], target[1], target[2], THRESHOLDS["position_mission_speed_m_s"],
                timeout_sec=THRESHOLDS["position_mission_timeout_seconds"], vehicle_name=vehicle).get())
            self.check(name + " command", result is True)
            state = self.state(name)
            row["error_m"] = distance(state["position"], target)
            row["actual_ned_m"] = state["position"]
            self.check(name + " position reached", row["error_m"] < THRESHOLDS["position_mission_error_m_lt"], row)
            self.check(name + " remains flying", state["landed_state"] == self.airsim.LandedState.Flying)
            row["completed"] = True
            self.save()
            return state

        target = [origin[0] + THRESHOLDS["position_mission_offset_m"], origin[1], origin[2]]
        before = position_mission("position offset", target)
        row = {"name": "velocity segment", "velocity_ned_m_s": [0.0, THRESHOLDS["velocity_mission_speed_m_s"], 0.0],
               "duration_seconds": THRESHOLDS["velocity_mission_seconds"], "completed": False}
        self.report["missions"].append(row)
        self.save()
        result = self.call("moveByVelocityAsync.get", lambda: self.client.moveByVelocityAsync(
            0.0, THRESHOLDS["velocity_mission_speed_m_s"], 0.0, THRESHOLDS["velocity_mission_seconds"],
            vehicle_name=vehicle).get())
        self.check("velocity mission command", result is True)
        self.check("post-velocity hover command", self.call("hoverAsync.get", lambda: self.client.hoverAsync(vehicle_name=vehicle).get()) is True)
        after = self.state("velocity segment")
        row.update(progress_m=after["position"][1] - before["position"][1],
                   displacement_m=distance(after["position"], before["position"]),
                   altitude_error_m=abs(after["position"][2] - before["position"][2]))
        self.check("velocity mission makes forward progress", row["progress_m"] > THRESHOLDS["velocity_mission_progress_m_gt"], row)
        self.check("velocity mission displacement bounded", row["displacement_m"] < THRESHOLDS["velocity_mission_displacement_m_lt"], row)
        self.check("velocity mission altitude bounded", row["altitude_error_m"] < THRESHOLDS["velocity_mission_altitude_error_m_lt"], row)
        self.check("velocity mission remains flying", after["landed_state"] == self.airsim.LandedState.Flying)
        row["completed"] = True
        self.save()
        position_mission("return above landing site", list(origin))

    def sensors(self):
        self.phase("sensors")
        vehicle = self.options["vehicle"]
        home = self.call("getHomeGeoPoint", lambda: self.client.getHomeGeoPoint(vehicle_name=vehicle))
        home_values = [float(home.latitude), float(home.longitude), float(home.altitude)]
        self.check("finite home geopoint", all(math.isfinite(x) for x in home_values), home_values)
        previous = None
        for index in range(THRESHOLDS["sensor_samples"]):
            if index:
                self.wait(THRESHOLDS["sensor_interval_seconds"])
            imu = self.call("getImuData", lambda: self.client.getImuData(imu_name="Imu", vehicle_name=vehicle))
            gps = self.call("getGpsData", lambda: self.client.getGpsData(gps_name="Gps", vehicle_name=vehicle))
            baro = self.call("getBarometerData", lambda: self.client.getBarometerData(barometer_name="Barometer", vehicle_name=vehicle))
            geo = gps.gnss.geo_point
            row = {"index": index, "timestamps": [int(imu.time_stamp), int(gps.time_stamp), int(baro.time_stamp)],
                   "imu": {"acceleration": vector(imu.linear_acceleration), "angular_velocity": vector(imu.angular_velocity),
                           "quaternion": quaternion(imu.orientation)},
                   "gps": {"valid": bool(gps.is_valid), "fix": int(gps.gnss.fix_type),
                           "position": [float(geo.latitude), float(geo.longitude), float(geo.altitude)],
                           "velocity": vector(gps.gnss.velocity), "eph": float(gps.gnss.eph), "epv": float(gps.gnss.epv)},
                   "barometer": {"pressure": float(baro.pressure), "altitude": float(baro.altitude), "qnh": float(baro.qnh)}}
            self.report["sensors"].append(row)
            self.save()
            i, g, b = row["imu"], row["gps"], row["barometer"]
            self.check("IMU finite", all(math.isfinite(x) for x in i["acceleration"] + i["angular_velocity"] + i["quaternion"]))
            self.check("IMU quaternion normalized", abs(sum(x*x for x in i["quaternion"]) - 1) < THRESHOLDS["quaternion_norm_squared_error_lt"])
            self.check("stationary IMU gravity range", THRESHOLDS["imu_acceleration_m_s2_gt"] < norm(i["acceleration"]) < THRESHOLDS["imu_acceleration_m_s2_lt"], norm(i["acceleration"]))
            self.check("GPS valid fix", g["valid"] and 2 <= g["fix"] <= 3, g)
            self.check("GPS finite", all(math.isfinite(x) for x in g["position"] + g["velocity"] + [g["eph"], g["epv"]]))
            self.check("GPS ranges", -90 <= g["position"][0] <= 90 and -180 <= g["position"][1] <= 180 and g["eph"] >= 0 and g["epv"] >= 0)
            self.check("GPS near home", all(abs(g["position"][j] - home_values[j]) < THRESHOLDS["gps_home_lat_lon_error_degrees_lt"] for j in (0, 1)))
            self.check("barometer finite", all(math.isfinite(x) for x in b.values()))
            self.check("barometer range", THRESHOLDS["barometer_pressure_pa_gt"] < b["pressure"] < THRESHOLDS["barometer_pressure_pa_lt"], b)
            self.check("sensor timestamps positive", all(t > 0 for t in row["timestamps"]), row["timestamps"])
            if previous is not None:
                self.check("all sensor timestamps advance", all(t > p for t, p in zip(row["timestamps"], previous)), row["timestamps"])
            previous = row["timestamps"]

    def images(self):
        self.phase("mixed_images")
        empty = self.call("simGetImages(empty)", lambda: self.client.simGetImages([], vehicle_name=self.options["vehicle"]))
        self.check("empty image request returns empty list", empty == [],
                   {"response_type": type(empty).__name__, "response_count": len(empty) if isinstance(empty, list) else None})
        assets = self.call("simListAssets", self.client.simListAssets)
        self.report["asset_query"] = {"count": len(assets) if isinstance(assets, list) else None,
                                      "sample": [str(asset) for asset in assets[:10]] if isinstance(assets, list) else []}
        self.check("asset registry returns nonempty asset list", isinstance(assets, list) and len(assets) > 0,
                   self.report["asset_query"])
        a, np, cv2 = self.airsim, self.np, self.cv2
        requests = [a.ImageRequest(self.options["camera"], a.ImageType.Scene, False, True),
                    a.ImageRequest(self.options["camera"], a.ImageType.DepthPlanar, True, False),
                    a.ImageRequest(self.options["camera"], a.ImageType.Scene, False, False)]
        width, height = THRESHOLDS["image_width"], THRESHOLDS["image_height"]
        previous = None
        for index in range(self.options["image_batches"]):
            if index:
                self.wait(THRESHOLDS["image_interval_seconds"])
            responses = self.call("simGetImages(mixed)", lambda: self.client.simGetImages(requests, vehicle_name=self.options["vehicle"]))
            row = {"index": index, "response_count": len(responses), "completed": False}
            self.report["images"].append(row)
            self.check("three mixed image responses", len(responses) == 3, len(responses))
            row["responses"] = [{"width": int(r.width), "height": int(r.height), "timestamp": int(r.time_stamp),
                                 "image_type": int(r.image_type), "pixels_as_float": bool(r.pixels_as_float),
                                 "compress": bool(r.compress), "message": str(r.message)} for r in responses]
            self.save()
            for j, (r, expected) in enumerate(zip(responses, requests)):
                self.check("image %s dimensions" % j, r.width == width and r.height == height, row["responses"][j])
                self.check("image %s request/response match" % j, r.image_type == expected.image_type and
                           bool(r.pixels_as_float) == bool(expected.pixels_as_float) and bool(r.compress) == bool(expected.compress))
                self.check("image %s no server error" % j, not r.message, str(r.message))
            stamps = [int(r.time_stamp) for r in responses]
            self.check("image timestamps positive", all(t > 0 for t in stamps), stamps)
            if previous is not None:
                self.check("all image timestamps advance", all(t > p for t, p in zip(stamps, previous)), stamps)
            previous = stamps
            png_bytes, raw_bytes = bytes(responses[0].image_data_uint8), bytes(responses[2].image_data_uint8)
            row["png_bytes"], row["raw_bytes"] = len(png_bytes), len(raw_bytes)
            self.check("PNG signature", png_bytes.startswith(b"\x89PNG\r\n\x1a\n"))
            decoded = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            self.check("PNG decoding and dimensions", decoded is not None and decoded.shape == (height, width, 3))
            unique = int(np.unique(decoded.reshape(-1, 3), axis=0).shape[0])
            row["png_unique_colors"] = unique
            self.check("PNG spatial variation", unique > THRESHOLDS["png_unique_colors_gt"], unique)
            self.check("raw Scene byte count", len(raw_bytes) == width * height * 3, len(raw_bytes))
            raw = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(height, width, 3)
            self.check("raw Scene decoding", raw.shape == (height, width, 3))
            depth = np.asarray(responses[1].image_data_float, dtype=np.float32)
            row["depth_values"] = int(depth.size)
            self.check("depth element count", depth.size == width * height, int(depth.size))
            depth = depth.reshape(height, width)
            finite_positive = float(np.mean(np.isfinite(depth) & (depth > 0)))
            row["depth_finite_positive_fraction"] = finite_positive
            self.check("depth entirely finite and positive", finite_positive == THRESHOLDS["depth_finite_positive_fraction"], finite_positive)
            row.update(depth_min=float(depth.min()), depth_max=float(depth.max()),
                       png_sha256=hashlib.sha256(png_bytes).hexdigest(), raw_sha256=hashlib.sha256(raw_bytes).hexdigest(),
                       depth_sha256=hashlib.sha256(depth.tobytes()).hexdigest(), completed=True)
            if index == 0:
                output = Path(self.options["output"])
                (output / "scene.png").write_bytes(png_bytes)
                np.save(str(output / "depth.npy"), depth, allow_pickle=False)
                np.save(str(output / "scene-raw.npy"), raw, allow_pickle=False)
                row["artifacts"] = ["scene.png", "depth.npy", "scene-raw.npy"]
            self.save()

    def metrics(self, label):
        pid = self.options.get("server_pid")
        if pid is None:
            return
        row = {"label": label, "pid": pid, "time": utc_now(), "diagnostic_only": True}
        self.report["metrics"].append(row)
        try:
            def collect():
                import psutil
                process = psutil.Process(pid)
                identity = process.create_time()
                if self.server_identity is not None and identity != self.server_identity:
                    raise RuntimeError("server PID was reused; refusing to sample a different process")
                self.server_identity = identity
                row["process_created"] = identity
                row["working_set_bytes"] = int(process.memory_info().rss)
                row["socket_count"] = len(process.net_connections(kind="inet"))
            self.call("optional_server_metrics", collect)
        except Exception as error:
            row["unavailable"] = repr(error)
            self.report["warnings"].append("Optional server metrics unavailable: " + repr(error))
        self.save()

    def run(self):
        self.phase("connect")
        self.connect()
        settings_string = self.call("getSettingsString", self.client.getSettingsString)
        self.report["actual_settings"] = json.loads(settings_string)
        self.check("ClockSpeed is 1", self.report["actual_settings"].get("ClockSpeed", 1) == 1)
        vehicle_settings = self.report["actual_settings"].get("Vehicles", {}).get(self.options["vehicle"], {})
        self.check("selected vehicle uses SimpleFlight", vehicle_settings.get("VehicleType") == "SimpleFlight")
        self.check("initial API control is not owned", self.call("isApiControlEnabled", lambda: self.client.isApiControlEnabled(vehicle_name=self.options["vehicle"])) is False)
        first = self.grounded("initial")
        ground = first["position"]
        self.wait(THRESHOLDS["state_sample_interval_seconds"])
        next_state = self.state("initial clock check")
        self.check("initial simulation clock advances", next_state["timestamp"] > first["timestamp"])
        self.metrics("initial")
        self.phase("acceptance_hover")
        self.report["initial_flight"] = self.flight("initial", ground, self.options["hover_seconds"], include_missions=True)
        self.save()
        self.sensors()
        self.images()
        self.metrics("after_images")
        vehicle = self.options["vehicle"]
        for index in range(1, self.options["cycles"] + 1):
            self.phase("reset_cycle_%02d" % index)
            row = {"index": index, "completed": False}
            self.report["cycles"].append(row)
            self.save()
            self.call("reset", self.client.reset)
            self.epoch = "reset_%02d" % index
            self.last_state_timestamp = None  # Reset can legitimately restart the simulation clock.
            self.wait(THRESHOLDS["reset_settle_seconds"])
            recovered = self.grounded("reset recovered")
            row["reset_position_error_m"] = distance(recovered["position"], ground)
            self.check("reset returns to original ground position", row["reset_position_error_m"] < THRESHOLDS["reset_position_error_m_lt"], row["reset_position_error_m"])
            self.check("reset releases API control", self.call("isApiControlEnabled", lambda: self.client.isApiControlEnabled(vehicle_name=vehicle)) is False)
            self.wait(THRESHOLDS["state_sample_interval_seconds"])
            advanced = self.state("reset clock check")
            self.check("clock advances after reset", advanced["timestamp"] > recovered["timestamp"])
            row["flight"] = self.flight(self.epoch, recovered["position"], THRESHOLDS["cycle_hover_seconds"])
            row["completed"] = True
            self.save()
            if index % THRESHOLDS["reconnect_every_cycles"] == 0:
                reconnect = {"after_cycle": index, "old_client_closed": False, "new_client_verified": False}
                self.report["reconnects"].append(reconnect)
                self.call("close_client_for_reconnect", lambda: close_client(self.client))
                self.client = None
                reconnect["old_client_closed"] = True
                self.save()
                self.connect()
                self.grounded("reconnect landed", ground)
                self.check("reconnect preserves released API control", self.call("isApiControlEnabled", lambda: self.client.isApiControlEnabled(vehicle_name=vehicle)) is False)
                reconnect["new_client_verified"] = True
                self.save()
            self.metrics("cycle_%02d" % index)
        self.check("requested reset cycles completed", sum(bool(c["completed"]) for c in self.report["cycles"]) == self.options["cycles"])
        self.check("requested mixed batches completed", sum(bool(i["completed"]) for i in self.report["images"]) == self.options["image_batches"])

    def cleanup(self):
        """Fresh client, land and confirm Landed BEFORE disarm; never reset in air."""
        self.deadline = self.final_deadline
        self.phase("cleanup")
        cleanup = self.report["cleanup"]
        cleanup["status"] = "running"
        self.save()
        safety = self.report["safety"]
        recovery = None
        try:
            if any(safety.values()):
                recovery = self.new_client()
                vehicle = self.options["vehicle"]
                def step(name, function):
                    result = self.call("cleanup:" + name, function)
                    cleanup["steps"].append({"name": name, "result": result if isinstance(result, (bool, int, float, str)) else None})
                    self.save()
                    return result
                step("ping", recovery.ping)
                step("cancelLastTask", lambda: recovery.cancelLastTask(vehicle_name=vehicle))
                step("enableApiControl(True)", lambda: recovery.enableApiControl(True, vehicle_name=vehicle))
                if safety["may_be_armed"] or safety["may_be_airborne"]:
                    landed_result = step("landAsync.get", lambda: recovery.landAsync(
                        timeout_sec=THRESHOLDS["land_timeout_seconds"], vehicle_name=vehicle).get())
                    # Even a false command result does not justify midair disarm.
                    until = min(self.deadline, self.clock() + THRESHOLDS["land_settle_timeout_seconds"])
                    while True:
                        state = step("getMultirotorState", lambda: recovery.getMultirotorState(vehicle_name=vehicle))
                        if state.landed_state == self.airsim.LandedState.Landed or self.clock() >= until:
                            break
                        self.wait(min(0.2, until - self.clock()))
                    if state.landed_state != self.airsim.LandedState.Landed:
                        raise ValidationFailure("cleanup cannot confirm Landed; disarm and control release withheld")
                    if step("armDisarm(False)", lambda: recovery.armDisarm(False, vehicle_name=vehicle)) is not True:
                        raise ValidationFailure("cleanup disarm failed")
                    self.safety(may_be_armed=False, may_be_airborne=False)
                    if landed_result is not True:
                        self.report["warnings"].append("Cleanup land returned false, but Landed was confirmed before disarm")
                step("enableApiControl(False)", lambda: recovery.enableApiControl(False, vehicle_name=vehicle))
                if step("isApiControlEnabled", lambda: recovery.isApiControlEnabled(vehicle_name=vehicle)) is not False:
                    raise ValidationFailure("cleanup could not release API control")
                self.safety(control_may_be_owned=False)
                cleanup["status"] = "passed"
            else:
                cleanup["status"] = "not_needed"
        except BaseException as error:
            cleanup["status"] = "failed"
            add_error(self.report, "cleanup", error)
            traceback.print_exc()
        finally:
            for name, client in (("recovery", recovery), ("main", self.client)):
                if client is not None:
                    try:
                        self.call("close_" + name + "_client", lambda c=client: close_client(c))
                        cleanup["steps"].append({"name": "close_" + name + "_client", "result": "closed"})
                    except BaseException as error:
                        cleanup["status"] = "failed"
                        add_error(self.report, "close_" + name + "_client", error)
            self.client = None
            self.save()


def worker_main(options, main_deadline, final_deadline, cleanup_only=False):
    # Child-local log also captures dependency/import failures; result.json already exists.
    with (Path(options["output"]) / "worker.log").open("a", encoding="utf-8", buffering=1) as log:
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            report = read_report(options["output"])
            report["worker_pid"] = os.getpid()
            runner = None
            try:
                airsim, np, cv2 = load_runtime()
                report["environment"].update(airsim_version=getattr(airsim, "__version__", "unknown"),
                    numpy_version=np.__version__, opencv_version=cv2.__version__,
                    airsim_module=str(Path(airsim.__file__).resolve()))
                runner = Runner(options, report, airsim, np, cv2, main_deadline, final_deadline)
                if not cleanup_only:
                    runner.run()
                    report["worker_outcome"] = "passed"
            except BaseException as error:
                add_error(report, "runtime" if not cleanup_only else "recovery_import", error)
                report["worker_outcome"] = "failed"
                traceback.print_exc()
            finally:
                if runner is not None:
                    runner.cleanup()
                else:
                    report["cleanup"]["status"] = "failed" if any(report["safety"].values()) else "not_needed"
                write_report(options["output"], report)


def stop_worker(process):
    """Only the child created by this runner, never a PID supplied by the user."""
    if process.is_alive():
        process.terminate()
        process.join(STOP_SECONDS / 2)
    if process.is_alive():
        process.kill()
        process.join(STOP_SECONDS / 2)
    if process.is_alive():
        raise RuntimeError("validation worker did not stop; refusing a concurrent cleanup writer")


def watchdog_reason(report, now, main_deadline, final_deadline, cleanup_only=False):
    active = report.get("active_operation")
    if active is not None and now >= active["deadline_monotonic"]:
        return "operation deadline exceeded: " + active["name"]
    in_cleanup = cleanup_only or report.get("phase") == "cleanup"
    if now >= (final_deadline if in_cleanup else main_deadline):
        return "cleanup deadline exceeded" if in_cleanup else "main wall-clock deadline exceeded"
    return None


def supervise(options, worker_target=worker_main):
    """Create a fresh evidence directory and enforce budgets from another process."""
    output = Path(options["output"])
    output.mkdir(parents=True, exist_ok=False)  # Never overwrite evidence, even an empty old directory.
    options = dict(options, output=str(output.resolve()))
    started = time.monotonic()
    final_deadline = started + options["max_seconds"] - STOP_SECONDS
    main_deadline = final_deadline - CLEANUP_SECONDS - STOP_SECONDS
    report = initial_report(options, started)
    write_report(output, report)
    context = multiprocessing.get_context("spawn")  # Portable; no inherited socket/event-loop state.
    process = None
    interrupted = False
    try:
        process = context.Process(target=worker_target, args=(options, main_deadline, final_deadline), name="airsimx-validation")
        process.start()
        while process.is_alive():
            report = read_report(output)
            reason = watchdog_reason(report, time.monotonic(), main_deadline, final_deadline)
            if reason:
                raise TimeoutError(reason)
            process.join(0.1)
        report = read_report(output)
        if process.exitcode != 0:
            raise RuntimeError("validation worker exited with code %s" % process.exitcode)
    except BaseException as error:
        interrupted = isinstance(error, KeyboardInterrupt)
        if process is not None:
            stop_worker(process)
        report = read_report(output)
        if report.get("active_operation"):
            report["active_operation"]["outcome"] = "interrupted_by_supervisor"
        report["active_operation"] = None
        add_error(report, "supervisor", error)
        report["worker_outcome"] = "failed"
        if not any(report["safety"].values()) and report["cleanup"]["status"] == "not_started":
            report["cleanup"]["status"] = "not_needed"
        write_report(output, report)
        # Killing a blocked client does not stop its in-flight server command. Use
        # a separate bounded client to cancel/land; no promise if the server died.
        if any(report["safety"].values()) and time.monotonic() < final_deadline:
            recovery = context.Process(target=worker_target, args=(options, main_deadline, final_deadline, True), name="airsimx-cleanup")
            try:
                recovery.start()
                while recovery.is_alive():
                    current = read_report(output)
                    reason = watchdog_reason(current, time.monotonic(), main_deadline, final_deadline, cleanup_only=True)
                    if reason:
                        raise TimeoutError(reason)
                    recovery.join(0.1)
                report = read_report(output)
                if recovery.exitcode != 0:
                    raise RuntimeError("recovery worker exited with code %s" % recovery.exitcode)
            except BaseException as cleanup_error:
                if recovery.pid is not None:
                    stop_worker(recovery)
                report = read_report(output)
                add_error(report, "recovery_supervisor", cleanup_error)
                report["cleanup"]["status"] = "failed"
                report["active_operation"] = None
        elif any(report["safety"].values()):
            report["cleanup"]["status"] = "failed"
            report["warnings"].append("No cleanup time remains; inspect and land the dedicated simulator manually")
    report["finished_utc"] = utc_now()
    report["elapsed_seconds"] = time.monotonic() - started
    passed = (report["worker_outcome"] == "passed" and not report["errors"]
              and report["cleanup"]["status"] in ("passed", "not_needed")
              and not any(report["safety"].values()))
    report["status"] = "passed" if passed else "failed"
    report["acceptance_passed"] = passed and report["profile"]["full_acceptance_counts"]
    report["phase"] = "finished"
    write_report(output, report)
    print("%s: %s" % (report["status"].upper(), output.resolve() / "result.json"))
    if not report["profile"]["full_acceptance_counts"]:
        print("Shortened smoke profile: this is NOT full v0.1 runtime acceptance.")
    if any(report["safety"].values()):
        print("WARNING: cleanup could not prove safe final state; inspect the dedicated simulator.", file=sys.stderr)
    return 130 if interrupted else (0 if passed else 1)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="RPC host (default: loopback)")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="SimpleFlight")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--output", "--output-dir", required=True, help="new, non-existing evidence directory")
    parser.add_argument("--hover-seconds", type=float, default=60.0)
    parser.add_argument("--cycles", type=int, default=20, help="reset/takeoff/hover/land cycles (default: 20)")
    parser.add_argument("--image-batches", type=int, default=10, help="mixed PNG/depth/raw requests (default: 10)")
    parser.add_argument("--server-pid", type=int, help="optional read-only psutil memory/socket diagnostics; never killed")
    parser.add_argument("--rpc-timeout", type=float, default=40.0, help="per-operation watchdog, 32..60 seconds")
    parser.add_argument("--max-seconds", type=float, default=1200.0, help="total wall budget including 90-second cleanup reserve")
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535) or not args.host.strip() or not args.vehicle.strip() or not args.camera.strip():
        parser.error("host, vehicle and camera must be nonempty; port must be 1..65535")
    if not math.isfinite(args.hover_seconds) or args.hover_seconds <= 0 or args.cycles < 1 or args.image_batches < 1:
        parser.error("hover-seconds must be finite and positive; cycles and image-batches must be >= 1")
    if not math.isfinite(args.rpc_timeout) or not 32 <= args.rpc_timeout <= 60:
        parser.error("rpc-timeout must be finite and 32..60 seconds (longer than land's command timeout)")
    minimum = CLEANUP_SECONDS + 2 * STOP_SECONDS + args.hover_seconds + 30
    if not math.isfinite(args.max_seconds) or args.max_seconds <= minimum:
        parser.error("max-seconds must exceed %.1f seconds to reserve required hover and cleanup time" % minimum)
    if args.server_pid is not None and args.server_pid <= 0:
        parser.error("server-pid must be positive")
    if Path(args.output).exists():
        parser.error("output directory already exists; choose a fresh directory")
    return vars(args)


def main(argv=None):
    options = parse_args(argv)
    try:
        return supervise(options)
    except (OSError, ValueError) as error:
        print("Cannot prepare or persist validation evidence: %s" % error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
