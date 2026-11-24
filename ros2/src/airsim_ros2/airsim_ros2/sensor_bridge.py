#!/usr/bin/env python3
"""ROS 2 Jazzy publisher for AirSim LiDAR, IMU and Scene-camera samples.

The bridge only reads sensors. It never sends arming, takeoff, pose or flight commands.
AirSim timestamps are copied into ROS headers and /clock; ground-truth vehicle state is
not queried or published by this node.
"""
from __future__ import annotations

import threading
import time
import math
import json

import airsim
import cv2
import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

from .sensor_conversions import (
    camera_info_from_projection,
    imu_from_airsim,
    pointcloud2_from_xyz,
    stamp_from_ns,
    static_sensor_transforms,
)


from .sensor_samples import ImuSampleSelector, SourceSampleFilter
from tf2_ros import StaticTransformBroadcaster


def sensor_qos_profile():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class AirSimRos2SensorBridge(Node):
    def __init__(self, *, client_factory=None, start_workers=True, **node_kwargs) -> None:
        super().__init__('airsim_sensor_bridge', **node_kwargs)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 41451)
        self.declare_parameter('vehicle_name', 'UAV_A')
        self.declare_parameter('lidar_name', 'Lidar1')
        self.declare_parameter('imu_name', 'Imu')
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('frame_prefix', 'uav_a')
        self.declare_parameter('lidar_topic', 'lidar/points')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('imu_raw_topic', '')
        self.declare_parameter('imu_target_hz_sim_time', 0.0)
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('lidar_poll_hz', 10.0)
        self.declare_parameter('imu_poll_hz', 200.0)
        self.declare_parameter('camera_poll_hz', 5.0)
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('rpc_timeout_s', 2.0)
        self.declare_parameter('reconnect_delay_s', 1.0)
        for sensor in ('lidar', 'imu', 'camera'):
            self.declare_parameter(sensor + '_translation', [0.0, 0.0, 0.0])
            self.declare_parameter(sensor + '_rotation',
                                   [0.5, -0.5, 0.5, -0.5] if sensor == 'camera' else [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('diagnostic_period_s', 5.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.vehicle = str(self.get_parameter('vehicle_name').value)
        self.lidar_name = str(self.get_parameter('lidar_name').value)
        self.imu_name = str(self.get_parameter('imu_name').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.prefix = str(self.get_parameter('frame_prefix').value).strip('/')
        self.publish_clock_enabled = bool(self.get_parameter('publish_clock').value)
        if not self.prefix:
            raise ValueError('frame_prefix must not be empty')
        self.camera_enabled = bool(self.get_parameter('publish_camera').value)
        self.diagnostic_period_s = float(self.get_parameter('diagnostic_period_s').value)

        self.rpc_timeout_s = float(self.get_parameter('rpc_timeout_s').value)
        self.reconnect_delay_s = float(self.get_parameter('reconnect_delay_s').value)
        for field in ('lidar_poll_hz', 'imu_poll_hz', 'camera_poll_hz', 'rpc_timeout_s',
                      'reconnect_delay_s', 'diagnostic_period_s'):
            value = float(self.get_parameter(field).value)
            if not math.isfinite(value) or value <= 0.0:
                super().destroy_node()
                raise ValueError(field + ' must be finite and positive')
        self._client_factory = client_factory or airsim.MultirotorClient
        self._selector = ImuSampleSelector(float(self.get_parameter('imu_target_hz_sim_time').value))
        extrinsics = {key: self.get_parameter(key).value for sensor in ('lidar', 'imu', 'camera')
                      for key in (sensor + '_translation', sensor + '_rotation')}
        transforms = static_sensor_transforms(self.prefix, **extrinsics)
        sensor_qos = sensor_qos_profile()
        self.lidar_pub = self.create_publisher(PointCloud2, str(self.get_parameter('lidar_topic').value), sensor_qos)
        self.imu_pub = self.create_publisher(Imu, str(self.get_parameter('imu_topic').value), sensor_qos)
        raw_imu_topic = str(self.get_parameter('imu_raw_topic').value).strip()
        self.imu_raw_pub = self.create_publisher(Imu, raw_imu_topic, sensor_qos) if raw_imu_topic else None
        self.image_pub = self.create_publisher(Image, str(self.get_parameter('image_topic').value), sensor_qos_profile())
        self.camera_info_pub = self.create_publisher(CameraInfo, str(self.get_parameter('camera_info_topic').value), sensor_qos)
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = (
            self.create_publisher(Clock, str(self.get_parameter('clock_topic').value), clock_qos)
            if self.publish_clock_enabled else None
        )
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(transforms)

        self._stop = threading.Event()
        self._rpc_clients_lock = threading.Lock()
        self._rpc_clients = {}
        self._clock_lock = threading.Lock()
        self._last_clock_ns = 0
        self._gates = {name: SourceSampleFilter() for name in ('lidar', 'imu', 'camera')}
        self._stats = {
            name: {
                'count': 0, 'duplicates': 0, 'last_report': time.monotonic(), 'reported_count': 0,
                'published': 0, 'reported_published': 0, 'empty': 0, 'last_width': 0, 'last_bytes': 0,
            }
            for name in ('lidar', 'imu', 'camera')
        }
        self._workers: list[threading.Thread] = []
        self._destroyed = False
        if start_workers:
            for name, callback in (('lidar', self._lidar_loop), ('imu', self._imu_loop), ('camera', self._camera_loop)):
                if name != 'camera' or self.camera_enabled:
                    self._start_worker(name, float(self.get_parameter(name + '_poll_hz').value), callback)
        self.get_logger().info(
            f'Read-only AirSim bridge started: {self.vehicle} @ {self.host}:{self.port}; '
            f'frames={self.prefix}/(body_flu,lidar_link,imu_link,camera_optical_frame)'
        )

    def _start_worker(self, name: str, rate_hz: float, callback) -> None:
        if rate_hz <= 0.0:
            raise ValueError(f'{name}_poll_hz must be positive')
        thread = threading.Thread(target=self._worker, args=(name, rate_hz, callback), name=f'airsim-{name}', daemon=True)
        self._workers.append(thread)
        thread.start()

    def _new_client(self, name):
        client = self._client_factory(ip=self.host, port=self.port, timeout_value=self.rpc_timeout_s)
        with self._rpc_clients_lock:
            if self._stop.is_set():
                client.close()
                raise RuntimeError('sensor bridge is stopping')
            self._rpc_clients[name] = client
        try:
            if not client.ping():
                raise ConnectionError('AirSim ping failed')
            if self._stop.is_set():
                raise RuntimeError('sensor bridge is stopping')
            if self.vehicle not in client.listVehicles():
                raise ValueError('configured vehicle is not present in AirSim: ' + self.vehicle)
            if self._stop.is_set():
                raise RuntimeError('sensor bridge is stopping')
            if name == 'lidar':
                settings = json.loads(client.getSettingsString())
                sensor = settings.get('Vehicles', {}).get(self.vehicle, {}).get('Sensors', {}).get(self.lidar_name, {})
                if sensor.get('DataFrame') != 'SensorLocalFrame':
                    raise ValueError('readonly bridge requires LiDAR DataFrame=SensorLocalFrame in vehicle Sensors')
            return client
        except Exception:
            self._release_client(name, client)
            raise

    def _release_client(self, name, client):
        with self._rpc_clients_lock:
            if self._rpc_clients.get(name) is client:
                del self._rpc_clients[name]
        client.close()

    def _worker(self, name: str, rate_hz: float, callback) -> None:
        period = 1.0 / rate_hz
        client = None
        next_call = time.monotonic()
        last_error_log = 0.0
        try:
            while not self._stop.is_set() and rclpy.ok(context=self.context):
                try:
                    if client is None:
                        client = self._new_client(name)
                    if self._stop.is_set():
                        break
                    callback(client)
                except Exception as exc:
                    if client is not None:
                        self._release_client(name, client)
                        client = None
                    if self._stop.is_set() or not rclpy.ok(context=self.context):
                        break
                    now = time.monotonic()
                    if now - last_error_log >= self.diagnostic_period_s:
                        self.get_logger().error(f'{name} RPC/publish error: {type(exc).__name__}: {exc}')
                        last_error_log = now
                    self._stop.wait(self.reconnect_delay_s)
                    next_call = time.monotonic()
                    continue
                next_call += period
                self._stop.wait(max(0.0, next_call - time.monotonic()))
                if time.monotonic() - next_call > period:
                    next_call = time.monotonic()
        finally:
            if client is not None:
                self._release_client(name, client)

    def _accept_timestamp(self, name: str, timestamp_ns: int) -> bool:
        if self._stop.is_set():
            return False
        gate = self._gates[name]
        if not gate.accept(timestamp_ns):
            self._stats[name]['duplicates'] = gate.rejected
            self._report_rate_if_due(name)
            return False
        self._stats[name]['count'] = gate.accepted
        if self.publish_clock_enabled:
            self._publish_clock(int(timestamp_ns))
        self._report_rate_if_due(name)
        return True

    def _publish_clock(self, timestamp_ns: int) -> None:
        stamp = stamp_from_ns(timestamp_ns)
        with self._clock_lock:
            if timestamp_ns <= self._last_clock_ns:
                return
            self._last_clock_ns = timestamp_ns
            msg = Clock()
            msg.clock = stamp
            self.clock_pub.publish(msg)

    def _report_rate_if_due(self, name: str) -> None:
        now = time.monotonic()
        stat = self._stats[name]
        elapsed = now - stat['last_report']
        if elapsed >= self.diagnostic_period_s:
            count = stat['count'] - stat['reported_count']
            rate = count / elapsed if elapsed > 0.0 else 0.0
            if name == 'lidar':
                published = stat['published'] - stat['reported_published']
                self.get_logger().info(
                    f'{name}: {rate:.2f} unique samples/s wall; published={published}; empty={stat["empty"]}; '
                    f'last_width={stat["last_width"]}; last_bytes={stat["last_bytes"]}; '
                    f'duplicates={stat["duplicates"]}'
                )
                stat['reported_published'] = stat['published']
            elif name == 'imu':
                published = stat['published'] - stat['reported_published']
                self.get_logger().info(
                    f'{name}: source_unique={rate:.2f} samples/s wall; '
                    f'published={published / elapsed:.2f} samples/s; duplicates={stat["duplicates"]}'
                )
                stat['reported_published'] = stat['published']
            else:
                self.get_logger().info(f'{name}: {rate:.2f} unique samples/s wall; duplicates={stat["duplicates"]}')
            stat['last_report'] = now
            stat['reported_count'] = stat['count']

    def _lidar_loop(self, client: airsim.MultirotorClient) -> None:
        data = client.getLidarData(lidar_name=self.lidar_name, vehicle_name=self.vehicle)
        if not self._accept_timestamp('lidar', data.time_stamp):
            return
        msg = pointcloud2_from_xyz(data.point_cloud, data.time_stamp, f'{self.prefix}/lidar_link')
        if msg.width > 0:
            self._stats['lidar']['published'] += 1
            self._stats['lidar']['last_width'] = int(msg.width)
            self._stats['lidar']['last_bytes'] = len(msg.data)
            self.lidar_pub.publish(msg)
        else:
            self._stats['lidar']['empty'] += 1

    def _imu_loop(self, client: airsim.MultirotorClient) -> None:
        data = client.getImuData(imu_name=self.imu_name, vehicle_name=self.vehicle)
        if not self._accept_timestamp('imu', data.time_stamp):
            return
        msg = imu_from_airsim(data, data.time_stamp, f'{self.prefix}/imu_link')
        if self.imu_raw_pub is not None:
            self.imu_raw_pub.publish(msg)
        if self._selector.select(data.time_stamp):
            self.imu_pub.publish(msg)
            self._stats['imu']['published'] += 1

    def _camera_loop(self, client: airsim.MultirotorClient) -> None:
        responses = client.simGetImages(
            [airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, True)],
            vehicle_name=self.vehicle,
        )
        if not responses:
            return
        response = responses[0]
        if not response.image_data_uint8 or not self._accept_timestamp('camera', response.time_stamp):
            return
        encoded = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError('AirSim returned an image that OpenCV could not decode')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        frame = f'{self.prefix}/camera_optical_frame'
        image = Image()
        image.header.stamp = stamp_from_ns(response.time_stamp)
        image.header.frame_id = frame
        image.height, image.width = int(rgb.shape[0]), int(rgb.shape[1])
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = image.width * 3
        image.data = rgb.tobytes()
        camera = client.simGetCameraInfo(self.camera_name, self.vehicle)
        if self._stop.is_set():
            return
        info = camera_info_from_projection(
            image.width, image.height, camera.proj_mat.matrix,
            response.time_stamp, frame,
        )
        self.image_pub.publish(image)
        self.camera_info_pub.publish(info)

    def destroy_node(self):
        if self._destroyed:
            return
        self._stop.set()
        with self._rpc_clients_lock:
            clients = list(self._rpc_clients.values())
        for client in clients:
            client.close()  # SDK close cancels outstanding calls, including the handshake.
        for worker in self._workers:
            worker.join(timeout=self.rpc_timeout_s + 1.0)
        if any(worker.is_alive() for worker in self._workers):
            raise RuntimeError('RPC worker did not stop; refusing to destroy live ROS publishers')
        self._destroyed = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirSimRos2SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
