"""End-to-end check of the ROS 2 AirSim bridge against a live simulator.

Run inside a sourced ROS 2 environment with airsim_ros2 installed. It waits
for telemetry, verifies ENU/NED consistency, calls takeoff, drives the
vehicle with cmd_vel_enu, and lands, recording every step as JSON.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from airsim_interfaces.srv import Land, Takeoff


class Probe(Node):
    def __init__(self, vehicle):
        super().__init__('airsim_bridge_check')
        prefix = '/' + vehicle
        self.odom = None
        self.odom_ned = None
        self.image = None
        self.images = 0
        self.create_subscription(Odometry, prefix + '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, prefix + '/odom_local_ned', self.on_ned, 10)
        # The bridge publishes frames best-effort (droppable), so the probe
        # must request a matching QoS or ROS will not connect the two.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, prefix + '/camera_0/image_raw', self.on_image, image_qos)
        self.vel_pub = self.create_publisher(Twist, prefix + '/cmd_vel_enu', 10)
        self.takeoff_client = self.create_client(Takeoff, prefix + '/takeoff')
        self.land_client = self.create_client(Land, prefix + '/land')

    def on_odom(self, message):
        self.odom = message

    def on_ned(self, message):
        self.odom_ned = message

    def on_image(self, message):
        self.image = message
        self.images += 1

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        raise TimeoutError(label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--vehicle', default='PX4SITL')
    parser.add_argument('--takeoff-timeout', type=float, default=45.0)
    parser.add_argument('--land-timeout', type=float, default=60.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'checks': [], 'samples': []}

    def save():
        (args.output / 'result.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    def check(name, value, detail=None):
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        save()
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    rclpy.init()
    node = Probe(args.vehicle)
    try:
        check('odom received', node.wait_for(lambda: node.odom is not None, 30, 'no odom'))
        check('ned odom received', node.wait_for(lambda: node.odom_ned is not None, 30, 'no ned odom'))

        # Camera frames: raw bgr8 at the configured rate, timestamped in
        # simulation time like the other topics.
        first_seen = node.images
        check('image received', node.wait_for(lambda: node.image is not None, 30, 'no image'),
              {'frames': first_seen})
        image = node.image
        check('image encoding is bgr8', image.encoding == 'bgr8', image.encoding)
        check('image dimensions are consistent',
              image.height > 0 and image.width > 0 and image.step == image.width * 3 and
              len(image.data) == image.height * image.width * 3,
              {'height': image.height, 'width': image.width, 'step': image.step,
               'bytes': len(image.data)})
        # A blank or uniform buffer would satisfy every check above while
        # carrying no rendered scene, so require actual pixel variation too.
        pixels = np.frombuffer(image.data, dtype=np.uint8)
        check('image has rendered content',
              float(pixels.std()) > 1.0 and np.unique(pixels).size > 8,
              {'std': round(float(pixels.std()), 2),
               'unique_values': int(np.unique(pixels).size)})
        check('image timestamp is set',
              image.header.stamp.sec > 0 or image.header.stamp.nanosec > 0,
              {'sec': image.header.stamp.sec, 'nanosec': image.header.stamp.nanosec})
        check('image frame is the camera optical frame',
              image.header.frame_id.endswith('_optical'), image.header.frame_id)
        before = node.images
        node.spin(2.0)
        check('images keep arriving', node.images - before >= 10,
              {'frames_in_2s': node.images - before})
        report['image'] = {'encoding': image.encoding, 'height': image.height,
                           'width': image.width, 'frames': node.images}
        # The ENU and NED odometry are two publishers, and delivery order
        # across publishers is not guaranteed: taking the last message of
        # each can pair samples one publish apart. A landed PX4 still
        # drifts a few millimetres per tick, which broke the exact
        # equality checks below. Both messages are built from one AirSim
        # sample, so wait until their header stamps agree and compare that
        # pair only.
        def same_sample():
            return (node.odom is not None and node.odom_ned is not None and
                    node.odom.header.stamp.sec == node.odom_ned.header.stamp.sec and
                    node.odom.header.stamp.nanosec == node.odom_ned.header.stamp.nanosec)
        check('ENU and NED odometry share one sample',
              node.wait_for(same_sample, 10.0, 'no matched ENU/NED sample'),
              {'sec': node.odom.header.stamp.sec, 'nanosec': node.odom.header.stamp.nanosec})
        enu, ned = node.odom, node.odom_ned
        p, q = enu.pose.pose.position, enu.pose.pose.orientation
        n = ned.pose.pose.position
        report['samples'].append({'enu': [p.x, p.y, p.z], 'ned': [n.x, n.y, n.z]})
        check('ENU east equals NED east (y)', abs(p.x - n.y) < 1e-6, [p.x, n.y])
        check('ENU north equals NED north (x)', abs(p.y - n.x) < 1e-6, [p.y, n.x])
        check('ENU up equals minus NED down', abs(p.z + n.z) < 1e-6, [p.z, n.z])
        check('orientation normalized', abs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1) < 1e-6)
        check('finite pose', all(math.isfinite(v) for v in (p.x, p.y, p.z, q.w, q.x, q.y, q.z)))

        base_up = node.odom.pose.pose.position.z
        check('takeoff service available', node.takeoff_client.wait_for_service(timeout_sec=20))
        request = Takeoff.Request()
        request.wait_on_last_task = True
        future = node.takeoff_client.call_async(request)
        node.wait_for(lambda: future.done(), args.takeoff_timeout, 'takeoff timeout')
        check('takeoff success', future.result().success is True)
        node.spin(2.0)
        # The vehicle's takeoff point is not the ENU origin, so compare against
        # the pre-takeoff height instead of an absolute threshold.
        climb = node.odom.pose.pose.position.z - base_up
        check('climbed in ENU up', climb > 1.0, climb)

        # The camera stream is what users actually consume while flying, so
        # verify it again in the air rather than trusting the ground sample.
        def stamp_ns(message):
            return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        ground_stamp_ns = stamp_ns(image)
        airborne_start_ns = stamp_ns(node.image)
        airborne_before = node.images
        node.spin(2.0)
        check('images keep arriving in flight', node.images - airborne_before >= 10,
              {'frames_in_2s': node.images - airborne_before})
        airborne_end_ns = stamp_ns(node.image)
        # Strict advance over the whole airborne window: a mere inequality
        # against the first ground frame would pass a frozen, replayed or
        # zeroed capture stamp. The capture clock must move at least a
        # second during the two-second window, and start after takeoff.
        check('image capture time advances in flight',
              airborne_start_ns > ground_stamp_ns and airborne_end_ns - airborne_start_ns >= 10**9,
              {'ground_ns': ground_stamp_ns, 'airborne_start_ns': airborne_start_ns,
               'airborne_end_ns': airborne_end_ns,
               'advanced_s': (airborne_end_ns - airborne_start_ns) / 1e9})

        start = (node.odom.pose.pose.position.x, node.odom.pose.pose.position.y)
        twist = Twist()
        twist.linear.x = 1.0  # east in ROS ENU
        # Keep commanding until the vehicle has clearly moved. A firmware
        # vehicle (PX4) has to re-enter Offboard mode after the takeoff/hover
        # handshake, which costs a couple of seconds that SimpleFlight does
        # not need, so a fixed short window under-measures the motion.
        deadline = time.monotonic() + 10.0
        track_start = None
        while time.monotonic() < deadline:
            node.vel_pub.publish(twist)
            node.spin(0.1)
            moved_east = node.odom.pose.pose.position.x - start[0]
            if track_start is None and moved_east > 0.1:
                # Measure cross-track from where eastward motion began: a
                # firmware vehicle drifts while it re-enters Offboard mode,
                # and that transition drift is not tracking error.
                track_start = node.odom.pose.pose.position.y
            if moved_east > 0.5:
                break
        node.spin(1.0)
        moved = (node.odom.pose.pose.position.x - start[0], node.odom.pose.pose.position.y - start[1])
        if track_start is None:
            track_start = start[1]
        cross_track = node.odom.pose.pose.position.y - track_start
        report['commanded_east_m'] = moved[0]
        report['cross_track_m'] = cross_track
        check('cmd_vel_enu moves east', moved[0] > 0.5, moved)
        check('cmd_vel_enu keeps north cross-track small', abs(cross_track) < 0.5,
              {'cross_track_m': cross_track, 'transition_drift_m': track_start - start[1]})

        check('land service available', node.land_client.wait_for_service(timeout_sec=20))
        land_request = Land.Request()
        land_request.wait_on_last_task = True
        future = node.land_client.call_async(land_request)
        node.wait_for(lambda: future.done(), args.land_timeout, 'land timeout')
        check('land success', future.result().success is True)
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        save()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
