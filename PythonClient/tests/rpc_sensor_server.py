"""Local sensor-only MessagePack server for transport and bridge regressions."""
import json
import socket
import socketserver
import threading
import time

import msgpack


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        fixture = self.server.fixture
        with fixture.lock:
            fixture.sockets.add(self.request)
            fixture.connections += 1
        parser = msgpack.Unpacker(raw=False)
        try:
            while True:
                data = self.request.recv(65536)
                if not data:
                    return
                parser.feed(data)
                for kind, identifier, method, params in parser:
                    if kind != 0:
                        return
                    with fixture.lock:
                        fixture.calls.append(method)
                    value = fixture.response(method)
                    if value is _NO_REPLY:
                        continue
                    self.request.sendall(msgpack.packb([1, identifier, None, value], use_bin_type=True))
        except OSError:
            pass
        finally:
            with fixture.lock:
                fixture.sockets.discard(self.request)


_NO_REPLY = object()
_VECTOR = {'x_val': 0., 'y_val': 0., 'z_val': 0.}
_QUATERNION = {'x_val': 0., 'y_val': 0., 'z_val': 0., 'w_val': 1.}
_POSE = {'position': _VECTOR, 'orientation': _QUATERNION}


class SensorRpcServer:
    def __init__(self, *, port=0, frame='SensorLocalFrame', stamp=1_000_000_000,
                 clock_speed=1, camera_timeout_once=False):
        self.frame, self.stamp, self.clock_speed = frame, stamp, clock_speed
        self.camera_timeout_once = camera_timeout_once
        self.calls, self.sockets, self.connections = [], set(), 0
        self.lock = threading.Lock()
        self.server = _Server(('127.0.0.1', port), _Handler)
        self.server.fixture = self
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': .01}, daemon=True)
        self.closed = False

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *unused):
        self.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.server.shutdown()
        with self.lock:
            sockets = list(self.sockets)
        for connection in sockets:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        self.server.server_close()
        self.thread.join(timeout=2)

    def response(self, method):
        if method == 'ping':
            return True
        if method == 'listVehicles':
            return ['UAV_A']
        if method == 'getSettingsString':
            return json.dumps({'ClockType': 'ScalableClock', 'ClockSpeed': self.clock_speed,
                               'Vehicles': {'UAV_A': {'Sensors': {
                                   'Lidar1': {'DataFrame': self.frame}}}}})
        if method == 'getLidarData':
            return {'time_stamp': self.stamp, 'point_cloud': [1., 2., 3.],
                    'pose': _POSE, 'segmentation': []}
        if method == 'getImuData':
            return {'time_stamp': self.stamp, 'orientation': _QUATERNION,
                    'angular_velocity': _VECTOR,
                    'linear_acceleration': {'x_val': 0., 'y_val': 0., 'z_val': -9.81}}
        if method == 'simGetImages':
            import cv2
            import numpy as np
            ok, png = cv2.imencode('.png', np.zeros((4, 8, 3), dtype=np.uint8))
            if not ok:
                raise RuntimeError('test PNG encoding failed')
            return [{'time_stamp': self.stamp, 'image_data_uint8': png.tobytes(),
                     'image_data_float': [], 'camera_position': _VECTOR,
                     'camera_orientation': _QUATERNION, 'message': '',
                     'pixels_as_float': False, 'compress': True,
                     'width': 8, 'height': 4, 'image_type': 0}]
        if method == 'simGetCameraInfo':
            if self.camera_timeout_once:
                self.camera_timeout_once = False
                return _NO_REPLY
            return {'fov': 90., 'pose': _POSE, 'proj_mat': {'matrix': [
                [0., 1., 0., 0.], [0., 0., -2., 0.],
                [0., 0., 0., 10.], [-1., 0., 0., 0.]]}}
        raise AssertionError('sensor-only fixture received unexpected RPC: ' + method)


def wait_for_idle_disconnect(client):
    """Observe the SDK reader detecting EOF; do not send a reconnecting ping."""
    transport = client.client
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        with transport._lock:
            if transport._pending:
                raise AssertionError('disconnect regression requires no in-flight requests')
            if transport._socket is None:
                return
        time.sleep(.005)
    raise AssertionError('SDK did not observe the idle socket disconnect')
