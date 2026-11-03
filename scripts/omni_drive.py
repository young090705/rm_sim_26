#!/usr/bin/env python3
"""底盘坐标系速度 /cmd_vel (Twist) -> 4x 轮速 (Float64).

输入 x 向底盘前方、y 向底盘左侧、z 向上，不随云台转动。
angular.z 是底盘角速度, 云台关节本身仍由独立的位置接口控制。

X 布局四全向轮, 轮位角 theta_i (base 系), 驱动方向为切向:
    w_i = SIGN * (-vx*sin(t) + vy*cos(t) + wz*R) / r

参数来自 CAD 实测: 安装半径 R=0.245 m, 轮半径 r=0.076 m.
SIGN 取决于轮系 z 轴朝向与 gz 关节正方向, 冒烟测试标定 (车动反则取反).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.076
MOUNT_RADIUS = 0.245
SIGN = -1.0

WHEELS = {
    'wheel_fl': math.radians(45.0),
    'wheel_rl': math.radians(135.0),
    'wheel_rr': math.radians(-135.0),
    'wheel_fr': math.radians(-45.0),
}


class OmniDrive(Node):
    def __init__(self, **kwargs):
        super().__init__('omni_drive', **kwargs)
        self.drive_type = self.declare_parameter('drive_type', 'omni').value
        if self.drive_type not in ('omni', 'mecanum'):
            raise ValueError('drive_type must be omni or mecanum')
        self.pubs = {
            name: self.create_publisher(Float64, f'/{name}/cmd_vel', 10)
            for name in WHEELS
        }
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        if self.drive_type == 'omni':
            speeds = {
                name: SIGN * (-vx * math.sin(theta) + vy * math.cos(theta)
                              + wz * MOUNT_RADIUS) / WHEEL_RADIUS
                for name, theta in WHEELS.items()
            }
        else:
            # Legacy mecanum geometry: wheelbase 0.40 m, separation 0.42 m.
            turn = wz * (0.40 + 0.42) / 2
            speeds = {
                'wheel_fl': (vx - vy - turn) / WHEEL_RADIUS,
                'wheel_rl': (vx + vy - turn) / WHEEL_RADIUS,
                'wheel_rr': (vx - vy + turn) / WHEEL_RADIUS,
                'wheel_fr': (vx + vy + turn) / WHEEL_RADIUS,
            }

        for name, speed in speeds.items():
            out = Float64()
            out.data = speed
            self.pubs[name].publish(out)


def main():
    rclpy.init()
    node = OmniDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
