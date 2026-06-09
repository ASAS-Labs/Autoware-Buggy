#!/usr/bin/env python3
"""
Vehicle interface stub with velocity feedback computed from EKF position.
Subscribes to /localization/kinematic_state, computes dx/dt, publishes as velocity_status.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

from autoware_vehicle_msgs.msg import (
    VelocityReport, SteeringReport, GearReport, ControlModeReport
)
from autoware_perception_msgs.msg import PredictedObjects
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class VehicleInterfaceStub(Node):
    def __init__(self):
        super().__init__('vehicle_interface_stub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE)

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE)

        # Vehicle status publishers
        self.pub_velocity = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', qos)
        self.pub_steering = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', qos)
        self.pub_gear = self.create_publisher(
            GearReport, '/vehicle/status/gear_status', qos)
        self.pub_control_mode = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', qos)

        # Dummy perception publishers
        self.pub_objects = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects', qos)
        self.pub_obstacle_pc = self.create_publisher(
            PointCloud2,
            '/perception/obstacle_segmentation/pointcloud', qos_be)

        # Subscribe to EKF output to compute velocity from position
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            qos_be)

        # State for velocity computation
        self.prev_x = None
        self.prev_y = None
        self.prev_t = None
        self.computed_velocity = 0.0

        # Timers
        self.create_timer(0.02, self.publish_vehicle_status)   # 50 Hz
        self.create_timer(0.1,  self.publish_perception_dummies)  # 10 Hz

        self.get_logger().info('Vehicle interface stub started (with position-based velocity feedback)')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_x is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 0.001:  # avoid division by zero
                dx = x - self.prev_x
                dy = y - self.prev_y
                dist = math.sqrt(dx*dx + dy*dy)
                # Low-pass filter to smooth velocity estimate
                raw_vel = dist / dt
                self.computed_velocity = 0.7 * self.computed_velocity + 0.3 * raw_vel

        self.prev_x = x
        self.prev_y = y
        self.prev_t = t

    def now_stamp(self):
        return self.get_clock().now().to_msg()

    def publish_vehicle_status(self):
        stamp = self.now_stamp()

        vel = VelocityReport()
        vel.header.stamp = stamp
        vel.header.frame_id = 'base_link'
        vel.longitudinal_velocity = float(self.computed_velocity)
        vel.lateral_velocity = 0.0
        vel.heading_rate = 0.0
        self.pub_velocity.publish(vel)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = 0.0
        self.pub_steering.publish(steer)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = GearReport.PARK
        self.pub_gear.publish(gear)

        ctrl = ControlModeReport()
        ctrl.stamp = stamp
        ctrl.mode = ControlModeReport.AUTONOMOUS
        self.pub_control_mode.publish(ctrl)

    def publish_perception_dummies(self):
        stamp = self.now_stamp()

        obj = PredictedObjects()
        obj.header.stamp = stamp
        obj.header.frame_id = 'map'
        obj.objects = []
        self.pub_objects.publish(obj)

        pc = PointCloud2()
        pc.header.stamp = stamp
        pc.header.frame_id = 'base_link'
        pc.height = 1
        pc.width = 0
        pc.is_dense = True
        pc.is_bigendian = False
        pc.point_step = 0
        pc.row_step = 0
        pc.data = []
        self.pub_obstacle_pc.publish(pc)

def main():
    rclpy.init()
    node = VehicleInterfaceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
