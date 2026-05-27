#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageFusionNode(Node):

    # ── 합성 파라미터 ──────────────────────────────────────────────
    # 열화상 최대 불투명도 (0.0~1.0)
    # 중심(뜨거운 곳)은 이 값, 경계는 강도에 비례해 낮아짐
    THERMAL_ALPHA = 0.72

    # 배경 파란색 틴트 강도 (0.0 = 순수 흑백, 1.0 = 강한 파랑)
    BLUE_TINT = 0.55

    # 파랑 배경 밝기 스케일 (1.0 = 원본 밝기 유지, 0.7 = 약간 어둡게)
    BG_BRIGHTNESS = 0.78

    def __init__(self):
        super().__init__('image_fusion')
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_thermal = None

        # Allow topic names to be overridden via ROS parameters
        self.declare_parameter('rgb_topic', '/camera/image_raw')
        self.declare_parameter('thermal_topic', '/thermal_image')
        self.declare_parameter('fused_topic', '/fused_image')

        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        self.fused_topic = self.get_parameter('fused_topic').get_parameter_value().string_value

        self.create_subscription(Image, self.rgb_topic,  self._on_rgb,     10)
        self.create_subscription(Image, self.thermal_topic, self._on_thermal, 10)
        self.pub = self.create_publisher(Image, self.fused_topic, 10)
        self.create_timer(0.1, self._fuse)
        self.get_logger().info('ImageFusion v3 ready [Blue NV + Smooth Blend]')

    def _on_rgb(self, msg):
        self.latest_rgb = msg

    def _on_thermal(self, msg):
        self.latest_thermal = msg

    def _fuse(self):
        if self.latest_rgb is None or self.latest_thermal is None:
            return

        try:
            rgb_rgba = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='rgba8')
            rgb = cv2.cvtColor(rgb_rgba, cv2.COLOR_RGBA2BGR)
            thermal = self.bridge.imgmsg_to_cv2(self.latest_thermal, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        if thermal.shape[:2] != rgb.shape[:2]:
            thermal = cv2.resize(thermal, (rgb.shape[1], rgb.shape[0]),
                                 interpolation=cv2.INTER_LINEAR)

        # ── 파란색 야간투시 배경 만들기 ───────────────────────────
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # BGR 채널별로 파란 틴트 적용
        # Blue 채널: 원본 gray + 파랑 부스트
        # Green 채널: gray 약하게
        # Red 채널: gray 더 약하게 (전체적으로 차갑고 푸른 느낌)
        t = self.BLUE_TINT
        b_ch = np.clip(gray * (0.6 + t * 0.5) + t * 35.0, 0, 255)
        g_ch = np.clip(gray * (0.7 - t * 0.15), 0, 255)
        r_ch = np.clip(gray * (0.55 - t * 0.25), 0, 255)

        bg_blue = np.stack([b_ch, g_ch, r_ch], axis=2).astype(np.float32)
        bg_blue *= self.BG_BRIGHTNESS

        # ── 열화상 픽셀별 알파 맵 ─────────────────────────────────
        thermal_f = thermal.astype(np.float32)

        # 열화상 강도: 각 픽셀의 총 밝기 (0 = 배경검정, 높을수록 뜨거움)
        intensity = thermal_f.sum(axis=2)

        # 부드러운 알파 전환: sqrt로 낮은 강도도 어느정도 보이게
        alpha_map = np.clip(intensity / (255.0 * 3.0 * 0.35), 0.0, 1.0)
        alpha_map = np.sqrt(alpha_map)                      # 0→0, 1→1, 감마 보정
        alpha_map = (alpha_map * self.THERMAL_ALPHA)[:, :, np.newaxis]

        # ── 알파 블렌딩 ───────────────────────────────────────────
        fused = (thermal_f * alpha_map + bg_blue * (1.0 - alpha_map)).astype(np.uint8)

        # ── HUD 부착 ──────────────────────────────────────────────
        fused = self._draw_hud(fused)

        out_msg = self.bridge.cv2_to_imgmsg(fused, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.latest_rgb.header.frame_id
        self.pub.publish(out_msg)

    @staticmethod
    def _draw_hud(img: np.ndarray) -> np.ndarray:
        legends = [
            ('NORMAL',  (0, 200,   0)),
            ('CAUTION', (0, 255, 255)),
            ('WARNING', (0, 140, 255)),
            ('DANGER',  (0,   0, 255)),
        ]
        x = 10
        for label, color in legends:
            cv2.rectangle(img, (x, 6), (x + 14, 20), color, -1)
            cv2.putText(img, label, (x + 18, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1, cv2.LINE_AA)
            x += 95
        return img


def main(args=None):
    rclpy.init(args=args)
    node = ImageFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()