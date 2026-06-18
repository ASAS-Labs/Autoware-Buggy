#!/bin/bash
source ~/autoware/install/setup.bash

echo "Starting OS0 driver..."
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.96.62 \
  lidar_mode:=1024x20 \
  point_type:=xyzir \
  "proc_mask:=IMU|PCL" \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  pub_static_tf:=false \
  viz:=false &
OUSTER_PID=$!
sleep 5

echo "Starting IMU relay..."
python3 ~/imu_relay.py &
IMU_PID=$!
sleep 1

echo "Starting XYZIRC converter..."
python3 ~/ouster_to_xyzirc.py &
CONV_PID=$!

echo "All nodes started. PIDs: ouster=$OUSTER_PID imu=$IMU_PID conv=$CONV_PID"
echo "Press Ctrl+C to stop all"

trap "kill $OUSTER_PID $IMU_PID $CONV_PID; exit" SIGINT SIGTERM
wait
