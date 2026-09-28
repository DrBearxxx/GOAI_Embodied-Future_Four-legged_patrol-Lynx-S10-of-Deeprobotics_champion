"""Production AIRY adapter. No motor, policy, map or navigation command publisher."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from airy_contract import normalize_points, transform

PREFIX = '/wym/navigation/lightning'


def stamp(m):
    return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec


class Bridge(Node):
    def __init__(self, epoch, sensor, status_file):
        super().__init__('goai_navigation_lightning_bridge')
        self.epoch, self.sensor, self.status_file = epoch, sensor, Path(status_file)
        self.counts = Counter()
        self.last_stamp = {}
        self.last_pose = None
        self.error = None
        reliable = lambda depth: QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)
        best = lambda depth: QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_pub = self.create_publisher(PointCloud2, PREFIX+'/input/points', reliable(32))
        self.imu_pub = self.create_publisher(Imu, PREFIX+'/input/imu', reliable(2000))
        self.state_pub = self.create_publisher(String, PREFIX+'/state', reliable(20))
        self.subs = [
            self.create_subscription(Imu, f'/wym/slam/{sensor}/imu', self.imu, best(2000)),
            self.create_subscription(PointCloud2, f'/wym/slam/{sensor}/points', self.cloud, best(32)),
            self.create_subscription(Odometry, PREFIX+'/estimator/odometry', self.pose, reliable(100))]
        self.create_timer(1., self.status)

    def ready(self):
        # Only ONE estimator in production. No comparison-service dependency.
        return self.cloud_pub.get_subscription_count() >= 1 and self.imu_pub.get_subscription_count() >= 1

    def ordered(self, stream, t):
        previous = self.last_stamp.get(stream)
        if previous is not None and t < previous-500_000_000:
            # The IMU/LiDAR device clock has restarted. Restart the pair, giving
            # it a new spatial epoch, instead of splicing filter states together.
            raise RuntimeError('SENSOR_CLOCK_RESTART:'+stream)
        if previous is not None and t <= previous:
            self.counts[stream+'_reordered'] += 1
            return False
        self.last_stamp[stream] = t
        return True

    def imu(self, m):
        self.counts['imu_received'] += 1
        values = [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                  m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        if not np.isfinite(values).all():
            self.counts['imu_nonfinite'] += 1
            return
        if not self.ordered('imu', stamp(m)) or not self.ready():
            return
        # Preserve native time, full acceleration and gyro range, including shocks.
        self.imu_pub.publish(m)
        self.counts['imu_forwarded'] += 1

    def cloud(self, m):
        self.counts['cloud_received'] += 1
        if not self.ordered('cloud', stamp(m)) or not self.ready():
            return
        try:
            points, stats = normalize_points(m)
        except (ValueError, TypeError, KeyError) as exc:
            self.counts['cloud_invalid'] += 1
            self.error = str(exc)
            return
        out = PointCloud2()
        out.header = m.header
        out.height, out.width = 1, len(points)
        out.point_step, out.row_step = 24, 24*len(points)
        out.is_bigendian, out.is_dense = False, True
        out.fields = [PointField(name=n, offset=o, datatype=k, count=1) for n, o, k in
            [('x',0,7), ('y',4,7), ('z',8,7), ('intensity',12,7), ('time',16,7), ('ring',20,4)]]
        out.data = points.tobytes()
        self.cloud_pub.publish(out)
        self.counts['cloud_forwarded'] += 1
        self.counts['invalid_points'] += stats['invalid_points']

    def pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        raw = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(raw).all() or np.linalg.norm(raw[3:]) < 1e-8:
            self.counts['pose_nonfinite'] += 1
            return
        if not self.ordered('pose', stamp(m)):
            return
        now = time.monotonic()
        payload = dict(schema='goai.lightning.odometry.v1', epoch=self.epoch, sensor=self.sensor,
            stamp_ns=stamp(m), T_world_imu=transform(raw[:3], raw[3:]).tolist(),
            published_mono=now)
        self.state_pub.publish(String(data=json.dumps(payload, allow_nan=False, separators=(',',':'))))
        self.last_pose = now
        self.counts['poses'] += 1

    def status(self):
        state = dict(epoch=self.epoch, sensor=self.sensor, monotonic=time.monotonic(),
            counts=dict(self.counts), last_pose_mono=self.last_pose, error=self.error,
            subscribers=dict(points=self.cloud_pub.get_subscription_count(), imu=self.imu_pub.get_subscription_count()))
        tmp = self.status_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(state))
        tmp.replace(self.status_file)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--epoch', required=True)
    p.add_argument('--sensor', choices=['front', 'rear'], required=True)
    p.add_argument('--status-file', required=True)
    args = p.parse_args()
    rclpy.init()
    node = Bridge(args.epoch, args.sensor, args.status_file)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
