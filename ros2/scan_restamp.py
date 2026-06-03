#!/usr/bin/env python3
"""
/scan 의 타임스탬프를 ROS 현재시각으로 다시 찍어 /scan_fixed 로 재발행.
Unity(Windows 시계)와 ROS2(WSL 시계)의 시간 차이로 costmap이 스캔을 버리는
'timestamp earlier than transform cache' 문제를 우회한다.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class ScanRestamp(Node):
    def __init__(self):
        super().__init__('scan_restamp')
        # Unity가 어떤 QoS로 쏘든 받도록 best_effort 구독
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        # costmap이 받도록 발행
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.get_logger().info('scan_restamp 시작: /scan -> /scan_fixed (타임스탬프 보정)')

    def cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
