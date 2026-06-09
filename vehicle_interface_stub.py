#!/usr/bin/env python3
"""
Vehicle Interface Stub for Autoware Universe — Gokart Platform

Velocity source: NDT pose (PoseStamped) position delta — direct scan matching output
Steering source: IMU yaw rate + bicycle model
Perception:      Real LiDAR pipeline handles obstacle detection — no dummy topics

Launch order:
  T1: OS0 driver
  T2: python3 ~/ouster_to_xyzirc.py
  T3: python3 ~/imu_relay.py
  T4: python3 ~/autoware/vehicle_interface_stub.py
  T5: Autoware launch
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

from autoware_vehicle_msgs.msg import (
    VelocityReport, SteeringReport, GearReport, ControlModeReport
)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

# -- Gokart parameters --------------------------------------------------------
WHEELBASE       = 1.29    # meters -- front axle to rear axle
MAX_STEER_RAD   = 1.8     # max steering angle
VEL_ALPHA       = 0.4     # low-pass filter for velocity
STEER_ALPHA     = 0.3     # low-pass filter for steering
MIN_VEL_THRESH  = 0.8     # below this m/s -> report zero
STOPPED_FRAMES  = 5       # consecutive low-vel frames before declaring stopped
MAX_DT          = 0.3     # ignore gaps larger than this (seconds)
MAX_VEL         = 10.0    # sanity cap m/s (~36 km/h)


class VehicleInterfaceStub(Node):
    def __init__(self):
        super().__init__('vehicle_interface_stub')

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE)

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE)

        qos_ndt = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE)

        # -- Publishers -------------------------------------------------------
        self.pub_velocity = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', qos_reliable)
        self.pub_steering = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', qos_reliable)
        self.pub_gear = self.create_publisher(
            GearReport, '/vehicle/status/gear_status', qos_reliable)
        self.pub_control_mode = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', qos_reliable)

        # -- Subscribers ------------------------------------------------------
        self.create_subscription(
            PoseStamped,
            '/localization/pose_estimator/pose',
            self.ndt_pose_callback,
            qos_ndt)

        self.create_subscription(
            Imu,
            '/sensing/imu/imu_data',
            self.imu_callback,
            qos_be)

        # -- State ------------------------------------------------------------
        self.prev_x            = None
        self.prev_y            = None
        self.prev_t            = None
        self.yaw_rate          = 0.0
        self.computed_velocity = 0.0
        self.computed_steering = 0.0
        self.stopped_count     = 0

        # -- Timers -----------------------------------------------------------
        self.create_timer(0.02, self.publish_vehicle_status)  # 50 Hz

        self.get_logger().info(
            'Vehicle stub started -- real LiDAR perception pipeline active')

    # -- Callbacks ------------------------------------------------------------

    def imu_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def ndt_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_t = t
            return

        dt = t - self.prev_t

        if dt <= 0.0 or dt > MAX_DT:
            self.prev_x = x
            self.prev_y = y
            self.prev_t = t
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        raw_vel = math.sqrt(dx*dx + dy*dy) / dt

        if raw_vel > MAX_VEL:
            self.get_logger().warn(f'NDT jump rejected: {raw_vel:.2f} m/s')
            self.prev_x = x
            self.prev_y = y
            self.prev_t = t
            return

        self.prev_x = x
        self.prev_y = y
        self.prev_t = t

        if raw_vel < MIN_VEL_THRESH:
            self.stopped_count += 1
            if self.stopped_count >= STOPPED_FRAMES:
                self.computed_velocity = 0.0
                self.computed_steering = 0.0
                self.yaw_rate = 0.0
                self.prev_x = None
                self.prev_y = None
                self.prev_t = None
        else:
            self.stopped_count = 0
            raw_steer = math.atan2(self.yaw_rate * WHEELBASE, raw_vel)
            raw_steer = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, raw_steer))
            self.computed_velocity = (
                (1 - VEL_ALPHA) * self.computed_velocity + VEL_ALPHA * raw_vel)
            self.computed_steering = (
                (1 - STEER_ALPHA) * self.computed_steering + STEER_ALPHA * raw_steer)

        self.get_logger().info(
            f'vel={self.computed_velocity:.3f} m/s  steer={math.degrees(self.computed_steering):.1f} deg',
            throttle_duration_sec=1.0)

    # -- Publishers -----------------------------------------------------------

    def now_stamp(self):
        return self.get_clock().now().to_msg()

    def publish_vehicle_status(self):
        stamp = self.now_stamp()

        vel = VelocityReport()
        vel.header.stamp = stamp
        vel.header.frame_id = 'base_link'
        vel.longitudinal_velocity = float(self.computed_velocity)
        vel.lateral_velocity = 0.0
        vel.heading_rate = float(self.yaw_rate)
        self.pub_velocity.publish(vel)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = float(self.computed_steering)
        self.pub_steering.publish(steer)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = (GearReport.DRIVE
                       if self.computed_velocity > MIN_VEL_THRESH
                       else GearReport.PARK)
        self.pub_gear.publish(gear)

        ctrl = ControlModeReport()
        ctrl.stamp = stamp
        ctrl.mode = ControlModeReport.AUTONOMOUS
        self.pub_control_mode.publish(ctrl)


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
