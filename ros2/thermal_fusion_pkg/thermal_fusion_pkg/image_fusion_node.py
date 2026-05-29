#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge


class ImageFusionNode(Node):

    # ── 합성 파라미터 (원본 유지) ───────────────────────────────────
    THERMAL_ALPHA = 0.72
    BLUE_TINT = 0.55
    BG_BRIGHTNESS = 0.78

    # ── 과열 감지 임계값 (Unity 기준과 동기화) ──────────────────────────
    CAUTION_THRES = 80.0
    WARNING_THRES = 100.0
    DANGER_THRES  = 120.0

    def __init__(self):
        super().__init__('image_fusion')
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_thermal = None

        # ── 구독 및 발행 정의 ─────────────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw',  self._on_rgb,     10)
        self.create_subscription(Image, '/thermal_image',     self._on_thermal, 10)
        
        # 1. Unity로부터 실시간 센서 온도 배열 구독
        self.create_subscription(Float32MultiArray, '/machine_temperatures', self._on_temperatures, 10)
        
        # 2. 로봇 자율 위험 판단용 얼럿 토픽 퍼블리셔
        self.alert_pub = self.create_publisher(String, '/thermal_alerts', 10)
        
        self.pub = self.create_publisher(Image, '/fused_image', 10)
        
        self.create_timer(0.1, self._fuse)
        self.get_logger().info('ImageFusion Node Ready [HUD Graph & Alert Publisher Fully Integrated]')

    def _on_rgb(self, msg):
        self.latest_rgb = msg

    def _on_thermal(self, msg):
        self.latest_thermal = msg

    # ── 로봇 자율 과열 감지 및 토픽 발행 콜백 함수 ──────────────────────
    def _on_temperatures(self, msg):
        if not msg.data:
            return

        # 전체 설비 중 최고 온도 추출
        max_temp = max(msg.data)
        
        # 온도별 행동 지침 문자열 판정
        guide_text = "정상 가동 상태"

        if max_temp >= self.DANGER_THRES:
            guide_text = "즉시 점검 및 대피 검토"
        elif max_temp >= self.WARNING_THRES:
            guide_text = "점검 요원 파견 권고"
        elif max_temp >= self.CAUTION_THRES:
            guide_text = "모니터링 강화 필요"

        # /thermal_alerts 토픽 백그라운드 발행
        alert_msg = String()
        alert_msg.data = guide_text
        self.alert_pub.publish(alert_msg)

    def _fuse(self):
        if self.latest_rgb is None or self.latest_thermal is None:
            return

        rgb_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
        thermal_img = self.bridge.imgmsg_to_cv2(self.latest_thermal, desired_encoding='bgr8')

        if rgb_img.shape != thermal_img.shape:
            thermal_img = cv2.resize(thermal_img, (rgb_img.shape[1], rgb_img.shape[0]))

        # ── 배경 Blue NV 처리 ───────────────────────────────────
        bg_gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        bg_blue = cv2.merge([
            (bg_gray * self.BG_BRIGHTNESS * (1.0 + self.BLUE_TINT)).clip(0, 255).astype(np.uint8),
            (bg_gray * self.BG_BRIGHTNESS * (1.0 - self.BLUE_TINT)).clip(0, 255).astype(np.uint8),
            (bg_gray * self.BG_BRIGHTNESS * (1.0 - self.BLUE_TINT)).clip(0, 255).astype(np.uint8)
        ])

        # ── 알파 맵 연산 ──────────────────────────────────────────
        thermal_f = thermal_img.astype(np.float32)
        gray_thermal = cv2.cvtColor(thermal_img, cv2.COLOR_BGR2GRAY)
        alpha_map = np.clip(gray_thermal / (255.0 * 3.0 * 0.35), 0.0, 1.0)
        alpha_map = np.sqrt(alpha_map)
        alpha_map = (alpha_map * self.THERMAL_ALPHA)[:, :, np.newaxis]

        # ── 알파 블렌딩 및 출력 ────────────────────────────────────
        fused = (thermal_f * alpha_map + bg_blue * (1.0 - alpha_map)).astype(np.uint8)

        # ── [복원] 상단 범례(HUD Legend) 그리기 로직 ───────────────────────
        fused = self._draw_origin_hud_legend(fused)

        out_msg = self.bridge.cv2_to_imgmsg(fused, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.latest_rgb.header.frame_id
        self.pub.publish(out_msg)

    # 원본 이미지 상단 범례 그리기 함수 정의
    def _draw_origin_hud_legend(self, img: np.ndarray) -> np.ndarray:
        # 상단 검은색 바 배경 투명 처리 또는 사각형 생성
        cv2.rectangle(img, (0, 0), (img.shape[1], 35), (15, 15, 15), -1)

        # 범례 데이터 (색상 배열은 BGR 구조)
        legends = [
            {"text": "NORMAL",  "color": (0, 180, 0)},     # 진녹색
            {"text": "CAUTION", "color": (0, 240, 240)},   # 노란색
            {"text": "WARNING", "color": (0, 120, 255)},   # 주황색
            {"text": "DANGER",  "color": (0, 0, 245)}      # 빨간색
        ]

        start_x = 12
        for leg in legends:
            # 색상 네모 그리기
            cv2.rectangle(img, (start_x, 11), (start_x + 18, 25), leg["color"], -1)
            # 텍스트 라벨 매핑 (영문이므로 깨지지 않음)
            cv2.putText(img, leg["text"], (start_x + 24, 22), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
            # 다음 범례를 위한 x 좌표 간격 띄우기
            start_x += 105

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