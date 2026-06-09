#!/usr/bin/env python3
"""
Extended vehicle interface stub for Autoware Universe.
Publishes all topics needed to clear emergency and enable AUTO mode.
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
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class VehicleInterfaceStub(Node):
    def __init__(self):
        super().__init__('vehicle_interface_stub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Vehicle status publishers
        self.pub_velocity = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', qos)
        self.pub_steering = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', qos)
        self.pub_gear = self.create_publisher(
            GearReport, '/vehicle/status/gear_status', qos)
        self.pub_control_mode = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', qos)

        # Dummy perception publishers — clears emergency
        self.pub_objects = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects', qos)
        self.pub_obstacle_pc = self.create_publisher(
            PointCloud2,
            '/perception/obstacle_segmentation/pointcloud', qos_be)

        # 50 Hz timer for vehicle status
        self.create_timer(0.02, self.publish_vehicle_status)
        # 10 Hz timer for perception dummies
        self.create_timer(0.1, self.publish_perception_dummies)

        self.get_logger().info('Vehicle interface stub started (with dummy perception)')

    def now_stamp(self):
        t = self.get_clock().now().to_msg()
        return t

    def publish_vehicle_status(self):
        stamp = self.now_stamp()

        vel = VelocityReport()
        vel.header.stamp = stamp
        vel.header.frame_id = 'base_link'
        vel.longitudinal_velocity = 0.0
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

        # Empty predicted objects
        obj = PredictedObjects()
        obj.header.stamp = stamp
        obj.header.frame_id = 'map'
        obj.objects = []
        self.pub_objects.publish(obj)

        # Empty obstacle segmentation pointcloud
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
