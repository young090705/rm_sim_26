#!/usr/bin/env python3
"""Convert the simulated PointCloud2 scan to Livox CustomMsg."""

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LivoxCustomBridge(Node):
    def __init__(self):
        super().__init__('livox_custom_bridge')
        self.publisher = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.subscription = self.create_subscription(
            PointCloud2, '/livox/lidar_raw', self.publish_custom_msg,
            qos_profile_sensor_data)

    def publish_custom_msg(self, cloud):
        available_fields = {field.name for field in cloud.fields}
        if not {'x', 'y', 'z'}.issubset(available_fields):
            self.get_logger().error('Input point cloud must contain x, y and z fields.')
            return

        fields = ['x', 'y', 'z']
        if 'intensity' in available_fields:
            fields.append('intensity')
        if 'ring' in available_fields:
            fields.append('ring')

        points = point_cloud2.read_points(cloud, field_names=fields, skip_nans=True)
        msg = CustomMsg()
        msg.header = cloud.header
        msg.timebase = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]

        for values in points:
            values = dict(zip(fields, values))
            point = CustomPoint()
            point.x = float(values['x'])
            point.y = float(values['y'])
            point.z = float(values['z'])
            point.reflectivity = max(0, min(255, round(float(values.get('intensity', 0.0)))))
            point.tag = 0
            point.line = max(0, min(255, int(values.get('ring', 0))))
            # Gazebo supplies an instantaneous scan without per-point timing.
            point.offset_time = 0
            msg.points.append(point)

        msg.point_num = len(msg.points)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LivoxCustomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
