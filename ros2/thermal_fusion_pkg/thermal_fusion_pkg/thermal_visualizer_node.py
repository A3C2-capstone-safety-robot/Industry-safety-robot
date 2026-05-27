#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from std_msgs.msg    import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge       import CvBridge

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


# ══════════════════════════════════════════════════════
#  설정값
# ══════════════════════════════════════════════════════
IMG_W = 640
IMG_H = 480

FOV_DEG = 60.0
_f      = (IMG_W / 2) / np.tan(np.radians(FOV_DEG / 2))
DEFAULT_K = np.array([
    [_f,  0,  IMG_W / 2],
    [0,  _f,  IMG_H / 2],
    [0,   0,          1],
], dtype=np.float64)

TEMP_MIN = 40.0
TEMP_MAX = 180.0

MAP_FRAME = 'map'
CAMERA_FRAME = 'thermal_camera_link'
CAMERA_INFO_TOPIC = '/thermal/camera_info'

MACHINE_IDS = ['machine_1', 'machine_2', 'machine_3', 'machine_4']

# ── 열화상 블롭 파라미터 ──────────────────────────────────
# 가우시안 블롭의 기본 시그마 (투영 바운딩박스 크기에 곱해지는 비율)
# 클수록 열이 넓게 퍼짐
BLOB_SIGMA_RATIO = 0.18

# 중심 블롭 가중치 (꼭짓점 블롭 대비 얼마나 강하게)
CENTER_WEIGHT = 1.8

# 꼭짓점 블롭 가중치
CORNER_WEIGHT = 0.45

# 최종 블러 커널 크기 (홀수, 클수록 더 부드럽게)
# 실제 열화상 카메라의 저해상도 번짐 효과
FINAL_BLUR_K = 15

# extents 축소 비율
EXTENTS_SCALE = 0.35
MAX_SIGMA_RATIO = 0.18
OFFSCREEN_MARGIN_RATIO = 0.5


# ══════════════════════════════════════════════════════
class ThermalVisualizer(Node):

    def __init__(self):
        super().__init__('thermal_visualizer')
        self.bridge = CvBridge()

        # Allow overriding frame and topic names via ROS parameters
        self.declare_parameter('map_frame', MAP_FRAME)
        self.declare_parameter('camera_frame', CAMERA_FRAME)
        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.K = DEFAULT_K.copy()
        self.img_w = IMG_W
        self.img_h = IMG_H

        self.machine_boxes: list = []
        self.temperatures:  list = []

        self.create_subscription(
            Float32MultiArray, '/machine_temperatures',
            self.on_temperatures, 10)
        self.create_subscription(
            Float32MultiArray, '/machine_world_positions',
            self.on_positions, 10)
        # Subscribe to CameraInfo topic specified by parameter
        self.create_subscription(
            CameraInfo, self.camera_info_topic,
            self.on_camera_info, 10)

        self.pub = self.create_publisher(Image, '/thermal_image', 10)
        self.create_timer(0.1, self.render)

        # 가우시안 LUT 캐시 (sigma → 2D kernel)
        self._gauss_cache: dict = {}

        self.get_logger().info(
            f'ThermalVisualizer v5 ready '
            f'[FOV={FOV_DEG}°, f={_f:.1f}px, MultiBlob mode, '
            f'map_frame={self.map_frame}, camera_frame={self.camera_frame}, '
            f'camera_info_topic={self.camera_info_topic}]'
        )

    # ── 콜백 ────────────────────────────────────────────────────────
    def on_temperatures(self, msg: Float32MultiArray):
        self.temperatures = list(msg.data)

    def on_positions(self, msg: Float32MultiArray):
        data = list(msg.data)
        n = len(data) // 6
        self.machine_boxes = [
            (
                (data[i*6],   data[i*6+1], data[i*6+2]),
                (data[i*6+3], data[i*6+4], data[i*6+5]),
            )
            for i in range(n)
        ]

    def on_camera_info(self, msg: CameraInfo):
        if len(msg.k) == 9:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.width > 0 and msg.height > 0:
            self.img_w = int(msg.width)
            self.img_h = int(msg.height)

    # ── 렌더 ────────────────────────────────────────────────────────
    def render(self):
        if not self.machine_boxes or not self.temperatures:
            return

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.camera_frame, self.map_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF 조회 실패: {e}', throttle_duration_sec=3.0)
            return

        fx, fy   = self.K[0, 0], self.K[1, 1]
        ppx, ppy = self.K[0, 2], self.K[1, 2]

        # 누적 heat map (float32, 0~1)
        heat_accum = np.zeros((self.img_h, self.img_w), dtype=np.float32)
        pixel_list = []
        for i, ((cx, cy, cz), (ex, ey, ez)) in enumerate(self.machine_boxes):
            if i >= len(self.temperatures):
                break

            norm_val = float(np.clip(
                (self.temperatures[i] - TEMP_MIN) / (TEMP_MAX - TEMP_MIN), 0, 1
            ))

            # ── 투영할 포인트 목록: 중심 + 8 꼭짓점 ──────────────────
            sex = ex * EXTENTS_SCALE
            sey = ey * EXTENTS_SCALE
            sez = ez * EXTENTS_SCALE

            points_map = [(cx, cy, cz)]  # 중심점 먼저
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        points_map.append(
                            (cx + sx * sex, cy + sy * sey, cz + sz * sez)
                        )

            # ── 투영 ──────────────────────────────────────────────────
            projected_pts = []
            center_px = None
            center_uv = None
            center_cam = None

            for j, (wx, wy, wz) in enumerate(points_map):
                pt_w = PointStamped()
                pt_w.header.frame_id = self.map_frame
                pt_w.point.x, pt_w.point.y, pt_w.point.z = wx, wy, wz
                pt_c = tf2_geometry_msgs.do_transform_point(pt_w, tf_stamped)

                opt_x = -pt_c.point.y
                opt_y = -pt_c.point.z
                opt_z =  pt_c.point.x

                if opt_z <= 0.05:
                    continue

                u = fx * opt_x / opt_z + ppx
                v = fy * opt_y / opt_z + ppy
                projected_pts.append((u, v))

                if j == 0:
                    center_cam = (pt_c.point.x, pt_c.point.y, pt_c.point.z)
                    center_uv = (u, v)
                    center_px = (int(u), int(v))

            pixel_list.append(center_px)

            if center_uv is None:
                self.get_logger().warn(
                    '중심점이 카메라 뒤에 있어 렌더링 건너뜀',
                    throttle_duration_sec=2.0
                )
                continue

            if len(projected_pts) < 2:
                continue

            if center_uv is not None:
                margin_x = self.img_w * OFFSCREEN_MARGIN_RATIO
                margin_y = self.img_h * OFFSCREEN_MARGIN_RATIO
                if (
                    center_uv[0] < -margin_x or center_uv[0] > self.img_w + margin_x or
                    center_uv[1] < -margin_y or center_uv[1] > self.img_h + margin_y
                ):
                    self.get_logger().warn(
                        f'중심점이 화면에서 너무 멀어 렌더링 건너뜀: '
                        f'uv=({center_uv[0]:.1f}, {center_uv[1]:.1f}), '
                        f'image=({self.img_w}, {self.img_h})',
                        throttle_duration_sec=2.0
                    )
                    continue

            pts_arr = np.array(projected_pts)  # (N, 2)

            # ── 투영된 bbox 크기로 sigma 결정 ─────────────────────────
            span_u = pts_arr[:, 0].max() - pts_arr[:, 0].min()
            span_v = pts_arr[:, 1].max() - pts_arr[:, 1].min()
            base_sigma = max(span_u, span_v) * BLOB_SIGMA_RATIO
            max_sigma = max(self.img_w, self.img_h) * MAX_SIGMA_RATIO
            base_sigma = float(np.clip(base_sigma, 8.0, max_sigma))

            # ── 각 포인트에 가우시안 블롭 배치 ───────────────────────
            center_proj = center_uv
            corner_projs = projected_pts[1:]

            # 중심 블롭 (더 크고 강하게)
            self._add_blob(heat_accum, center_proj,
                           sigma=base_sigma * 1.1,
                           weight=CENTER_WEIGHT * norm_val)

            # 꼭짓점 블롭 (작고 약하게 — 형태 다양성)
            corner_sigma = base_sigma * 0.7
            for cp in corner_projs:
                self._add_blob(heat_accum, cp,
                               sigma=corner_sigma,
                               weight=CORNER_WEIGHT * norm_val)

        # ── 전체 블러로 열 번짐 효과 (실제 열화상 카메라 흉내) ────────
        if FINAL_BLUR_K > 1:
            heat_accum = cv2.GaussianBlur(
                heat_accum, (FINAL_BLUR_K, FINAL_BLUR_K), 0
            )

        # ── 0~1 클리핑 및 컬러맵 적용 ────────────────────────────────
        heat_accum = np.clip(heat_accum, 0.0, 1.0)
        gray8    = (heat_accum * 255).astype(np.uint8)
        colormap = cv2.applyColorMap(gray8, cv2.COLORMAP_JET)
        colormap[gray8 == 0] = (0, 0, 0)

        # ── 라벨 오버레이 ─────────────────────────────────────────────
        for i, pix in enumerate(pixel_list):
            if pix is None or i >= len(self.temperatures):
                continue
            u, v = pix
            if not (0 <= u < self.img_w and 0 <= v < self.img_h):
                continue
            t    = self.temperatures[i]
            name = MACHINE_IDS[i] if i < len(MACHINE_IDS) else f'M{i+1}'
            cv2.circle(colormap, (u, v), 5, self._status_color(t), -1)
            cv2.putText(
                colormap, f'{name}: {t:.1f}C',
                (u + 7, v - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (255, 255, 255), 1, cv2.LINE_AA
            )

        img_msg = self.bridge.cv2_to_imgmsg(colormap, encoding='bgr8')
        img_msg.header.stamp    = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.camera_frame
        self.pub.publish(img_msg)

    # ── 가우시안 블롭 추가 ───────────────────────────────────────────
    def _add_blob(self, canvas: np.ndarray, center, sigma: float, weight: float):
        """
        canvas (H,W float32) 의 center 위치에 가우시안 블롭을 더한다.
        화면 밖 중심이라도 번짐이 화면 안에 들어오면 정상 렌더링.
        """
        cx, cy = center
        H, W = canvas.shape

        # 블롭이 영향을 미치는 범위 (3-sigma)
        r = int(sigma * 3.5)
        x0 = int(cx) - r;  x1 = int(cx) + r + 1
        y0 = int(cy) - r;  y1 = int(cy) + r + 1

        # 캔버스와 교차 영역
        cx0 = max(x0, 0);  cx1 = min(x1, W)
        cy0 = max(y0, 0);  cy1 = min(y1, H)
        if cx0 >= cx1 or cy0 >= cy1:
            return

        # 교차 영역의 픽셀 좌표
        xs = np.arange(cx0, cx1, dtype=np.float32) - cx
        ys = np.arange(cy0, cy1, dtype=np.float32) - cy
        xg, yg = np.meshgrid(xs, ys)

        gauss = np.exp(-(xg**2 + yg**2) / (2.0 * sigma**2))
        canvas[cy0:cy1, cx0:cx1] += gauss * weight

    @staticmethod
    def _status_color(temp: float):
        if   temp >= 120: return (0,   0, 255)
        elif temp >= 100: return (0, 140, 255)
        elif temp >= 80:  return (0, 255, 255)
        else:             return (0, 200,   0)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
