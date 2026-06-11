#!/usr/bin/env python3
"""
map_relay.py — 저장된 맵을 Unity 미니맵으로 중계.

map_server는 /map 을 latched(TRANSIENT_LOCAL)로 '한 번만' 발행한다.
Unity ROS-TCP-Connector 구독은 latched 메시지를 못 받으므로, SLAM을 끄고
map_server만 쓰면 Unity 미니맵이 빈 화면이 된다.

이 노드는 /map 을 TRANSIENT_LOCAL로 구독해 최신 맵을 받아두고,
Unity가 받을 수 있는 일반(VOLATILE) QoS로 /map_unity 에 1Hz로 재발행한다.

  map_server ──(/map, latched)──> [map_relay] ──(/map_unity, 1Hz)──> Unity 미니맵

실행:
    python3 map_relay.py
Unity 쪽:
    MinimapController 의 Map Topic 을 /map_unity 로 변경.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapRelay(Node):
    def __init__(self):
        super().__init__("map_relay")

        self.declare_parameter("in_topic", "/map")
        self.declare_parameter("out_topic", "/map_unity")
        self.declare_parameter("publish_rate", 1.0)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        rate = float(self.get_parameter("publish_rate").value)

        # 입력: map_server의 latched 맵을 받기 위해 TRANSIENT_LOCAL 구독
        in_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        # 출력: Unity가 받기 쉬운 일반 VOLATILE/RELIABLE
        out_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._latest = None
        self.create_subscription(OccupancyGrid, in_topic, self._on_map, in_qos)
        self._pub = self.create_publisher(OccupancyGrid, out_topic, out_qos)
        self.create_timer(1.0 / max(0.1, rate), self._tick)

        self.get_logger().info(
            f"map_relay 시작: {in_topic} (latched) → {out_topic} ({rate}Hz)"
        )

    def _on_map(self, msg: OccupancyGrid):
        first = self._latest is None
        self._latest = msg
        if first:
            self.get_logger().info(
                f"맵 수신: {msg.info.width}x{msg.info.height} "
                f"@ {msg.info.resolution:.3f}m/cell — 재발행 시작"
            )

    def _tick(self):
        if self._latest is not None:
            # 타임스탬프만 갱신해 재발행 (Unity가 매번 새 메시지로 인식)
            self._latest.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._latest)


def main(args=None):
    rclpy.init(args=args)
    node = MapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
