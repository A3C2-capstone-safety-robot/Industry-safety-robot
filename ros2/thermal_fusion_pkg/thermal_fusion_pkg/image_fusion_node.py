#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ImageFusionNode(Node):

    THERMAL_ALPHA = 0.72
    BLUE_TINT = 0.55
    BG_BRIGHTNESS = 0.78

    CAUTION_THRES = 80.0
    WARNING_THRES = 100.0
    DANGER_THRES  = 120.0

    def __init__(self):
        super().__init__('image_fusion')
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_thermal = None

        # 카메라 장비 및 센서가 보내주는 토픽만 구독 (현실성 확보)
        self.create_subscription(Image, '/camera/image_raw',  self._on_rgb,     10)
        self.create_subscription(Image, '/thermal_image',     self._on_thermal, 10)
        self.create_subscription(String, '/thermal/raw_values', self._on_raw_thermal_data, 10)
        
        self.alert_pub = self.create_publisher(String, '/thermal_alerts', 10)
        self.pub = self.create_publisher(Image, '/fused_image', 10)
        
        self.create_timer(0.1, self._fuse)
        self.get_logger().info('ImageFusion Node Ready [Multi-Alert System Integrated]')

    def _on_rgb(self, msg):
        self.latest_rgb = msg

    def _on_thermal(self, msg):
        self.latest_thermal = msg

    def _on_raw_thermal_data(self, msg):
        try:
            # 가상 열화상 카메라(Visualizer)가 화각 내 장비만 골라 보낸 JSON 파싱
            visible_machines = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'JSON 파싱 실패: {e}')
            return
        
        if not visible_machines:
            self._publish_alert("None", 0.0)
            return

        alerts_triggered = False
        
        # 카메라 렌즈에 포착된 설비들만 루프를 돌며 경보 체크
        for m_id, m_temp in visible_machines.items():
            if m_temp >= self.CAUTION_THRES:
                self._publish_alert(m_id, float(m_temp))
                alerts_triggered = True

        if not alerts_triggered:
            alert_msg = String()
            alert_msg.data = "모든 감지 설비 온도 정상"
            self.alert_pub.publish(alert_msg)

    def _publish_alert(self, machine_id, max_temp):
        if machine_id == "None":
            alert_msg = String()
            alert_msg.data = "화면에 감지된 설비 없음 (정상)"
            self.alert_pub.publish(alert_msg)
            return

        guide_text = "정상 가동 상태"
        if max_temp >= self.DANGER_THRES:
            guide_text = "즉시 점검 및 대피 검토"
        elif max_temp >= self.WARNING_THRES:
            guide_text = "점검 요원 파견 권고"
        elif max_temp >= self.CAUTION_THRES:
            guide_text = "모니터링 강화 필요"

        final_alert_string = f"[{machine_id}] {max_temp:.1f}C - {guide_text}"
        
        alert_msg = String()
        alert_msg.data = final_alert_string
        self.alert_pub.publish(alert_msg)

    def _fuse(self):
        if self.latest_rgb is None or self.latest_thermal is None:
            return

        rgb_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
        thermal_img = self.bridge.imgmsg_to_cv2(self.latest_thermal, desired_encoding='bgr8')

        if rgb_img.shape != thermal_img.shape:
            thermal_img = cv2.resize(thermal_img, (rgb_img.shape[1], rgb_img.shape[0]))

        bg_gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        bg_blue = cv2.merge([
            (bg_gray * self.BG_BRIGHTNESS * (1.0 + self.BLUE_TINT)).clip(0, 255).astype(np.uint8),
            (bg_gray * self.BG_BRIGHTNESS * (1.0 - self.BLUE_TINT)).clip(0, 255).astype(np.uint8),
            (bg_gray * self.BG_BRIGHTNESS * (1.0 - self.BLUE_TINT)).clip(0, 255).astype(np.uint8)
        ])

        thermal_f = thermal_img.astype(np.float32)
        gray_thermal = cv2.cvtColor(thermal_img, cv2.COLOR_BGR2GRAY)
        alpha_map = np.clip(gray_thermal / (255.0 * 3.0 * 0.35), 0.0, 1.0)
        alpha_map = np.sqrt(alpha_map)
        alpha_map = (alpha_map * self.THERMAL_ALPHA)[:, :, np.newaxis]

        fused = (thermal_f * alpha_map + bg_blue * (1.0 - alpha_map)).astype(np.uint8)
        fused = self._draw_origin_hud_legend(fused)

        out_msg = self.bridge.cv2_to_imgmsg(fused, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.latest_rgb.header.frame_id
        self.pub.publish(out_msg)

    def _draw_origin_hud_legend(self, img: np.ndarray) -> np.ndarray:
        cv2.rectangle(img, (0, 0), (img.shape[1], 35), (15, 15, 15), -1)

        legends = [
            {"text": "NORMAL",  "color": (0, 180, 0)},     
            {"text": "CAUTION", "color": (0, 240, 240)},   
            {"text": "WARNING", "color": (0, 120, 255)},   
            {"text": "DANGER",  "color": (0, 0, 245)}      
        ]

        start_x = 12
        for leg in legends:
            cv2.rectangle(img, (start_x, 11), (start_x + 18, 25), leg["color"], -1)
            cv2.putText(img, leg["text"], (start_x + 24, 22), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
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