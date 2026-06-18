#!/usr/bin/env python3
"""
Converts Ouster OS0 pointcloud to Autoware's contiguous XYZI format.
Fields: x(4) y(4) z(4) intensity(4) = 16 bytes, NO gap between z and intensity.
This is what autoware_pointcloud_preprocessor crop_box_filter accepts.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class OusterToXYZIRC(Node):
    def __init__(self):
        super().__init__('ouster_to_xyzirc')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            qos_out
        )

        self.get_logger().info('Ouster → contiguous XYZI converter started')

    def callback(self, msg):
        try:
            fields = {f.name: f for f in msg.fields}

            data = np.frombuffer(msg.data, dtype=np.uint8)
            n_points = msg.width * msg.height
            point_step = msg.point_step

            if n_points == 0:
                return

            x_off = fields['x'].offset if 'x' in fields else 0
            y_off = fields['y'].offset if 'y' in fields else 4
            z_off = fields['z'].offset if 'z' in fields else 8
            i_off = fields['intensity'].offset if 'intensity' in fields else 16
            r_off = fields['ring'].offset if 'ring' in fields else 20
    

            points = data.reshape(n_points, point_step)
            # PointXYZIRC exact layout:
            # x:           float32  offset 0
            # y:           float32  offset 4
            # z:           float32  offset 8
            # intensity:   uint8    offset 12
            # return_type: uint8    offset 13
            # channel:     uint16   offset 14
            # point_step = 16
            out_step = 16
            out_data = np.zeros((n_points, out_step), dtype=np.uint8)

            # x, y, z as float32
            out_data[:, 0:4] = points[:, x_off:x_off+4]
            out_data[:, 4:8] = points[:, y_off:y_off+4]
            out_data[:, 8:12] = points[:, z_off:z_off+4]

            # intensity as uint8 (scale float intensity 0-255)
            raw_intensity = points[:, i_off:i_off+4].copy().view(np.float32).reshape(-1)
            intensity_u8 = np.clip(raw_intensity, 0, 255).astype(np.uint8)
            out_data[:, 12] = intensity_u8

            # return_type = 0
            out_data[:, 13] = 0

            # channel = ring as uint16
            if 'ring' in fields:
                rings = np.ascontiguousarray(points[:, r_off:r_off+2]).view(np.uint16).reshape(-1)
                out_data[:, 14:16] = np.ascontiguousarray(rings.view(np.uint8).reshape(-1, 2))

            out = PointCloud2()
            out.header = msg.header
            out.header.frame_id = 'os_lidar'
            out.height = 1
            out.width = n_points
            out.is_dense = False
            out.is_bigendian = False
            out.point_step = out_step
            out.row_step = out_step * n_points

            out.fields = [
                PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity',   offset=12, datatype=PointField.UINT8,   count=1),
                PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
                PointField(name='channel',     offset=14, datatype=PointField.UINT16,  count=1),
            ]

            out.data = out_data.flatten().tobytes()
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')

def main():
    rclpy.init()
    node = OusterToXYZIRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
