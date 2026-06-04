#!/usr/bin/env python3

import json
import re
import threading
import time as _time
import urllib.error
import urllib.request
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Float32MultiArray, String


GAS_ALIASES = {
    "NH3": "NH3",
    "AMMONIA": "NH3",
    "암모니아": "NH3",
    "H2S": "H2S",
    "HYDROGEN_SULFIDE": "H2S",
    "황화수소": "H2S",
    "CH4": "CH4",
    "METHANE": "CH4",
    "메탄": "CH4",
    "LNG": "CH4",
}


class LlmBridgeNode(Node):
    def __init__(self):
        super().__init__("llm_bridge")

        self.declare_parameter("backend_url", "http://127.0.0.1:8000/sensor")
        self.declare_parameter("post_interval_sec", 1.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("default_location", "Unknown")
        self.declare_parameter("default_gas_type", "UNKNOWN")
        self.declare_parameter("gas_concentration_topic", "/gas_concentration")
        self.declare_parameter("gas_alert_topic", "/gas_alert")
        self.declare_parameter("gas_type_topic", "/gas_type")
        self.declare_parameter("machine_temperatures_topic", "/machine_temperatures")
        self.declare_parameter("thermal_alert_topic", "/thermal_alerts")
        self.declare_parameter("odom_topic", "/odom")
        # ── 사건(이벤트) 토픽 — 리포트의 핵심 재료 ──
        self.declare_parameter("robot_status_topic", "/robot_status")
        self.declare_parameter("robot_mode_topic", "/robot_mode")
        self.declare_parameter("moth_result_topic", "/moth_search/result")
        self.declare_parameter("max_status_events", 30)
        # 기본 구역 정의 — patrol_navigator.py의 ZONE_DEFINITIONS와 동일 경계
        #   A구역: x<0, y<-5 / B구역: x<0, y>=-5 / C구역: 0<=x<21, y<0
        #   D구역: 0<=x<21, y>=0 / E구역: x>=21
        # (반평면을 ±100m 한계의 사각형으로 표현. 맵 바뀌면 파라미터로 덮어쓰기)
        default_zones = json.dumps([
            {"name": "A구역", "x_min": -100, "x_max": 0,   "y_min": -100, "y_max": -5},
            {"name": "B구역", "x_min": -100, "x_max": 0,   "y_min": -5,   "y_max": 100},
            {"name": "C구역", "x_min": 0,    "x_max": 21,  "y_min": -100, "y_max": 0},
            {"name": "D구역", "x_min": 0,    "x_max": 21,  "y_min": 0,    "y_max": 100},
            {"name": "E구역", "x_min": 21,   "x_max": 100, "y_min": -100, "y_max": 100},
        ], ensure_ascii=False)
        self.declare_parameter("zone_rectangles", default_zones)
        # Use sensor-data QoS (BEST_EFFORT) for subscriptions. Many simulators /
        # ROS bridges publish sensor topics as BEST_EFFORT, which would otherwise
        # silently mismatch the default RELIABLE profile and deliver no messages.
        self.declare_parameter("use_sensor_qos", True)

        self.backend_url = self.get_parameter("backend_url").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.default_location = self.get_parameter("default_location").value
        self.default_gas_type = self.get_parameter("default_gas_type").value
        self.zone_rectangles = self._load_zone_rectangles(
            self.get_parameter("zone_rectangles").value
        )

        self.latest_gas_concentration = None
        self.latest_gas_alert = None
        self.latest_gas_type = None
        self.latest_machine_temperatures = []
        self.latest_thermal_alert = None
        self.latest_odom_xy = None
        self.latest_robot_mode = None
        self.latest_moth_result = None
        # 최근 상태 보고 타임라인 — 리포트 생성용 ([{"time":..., "text":...}, ...])
        self.status_events = deque(
            maxlen=int(self.get_parameter("max_status_events").value)
        )
        self._last_payload_json = None

        # Guards shared sensor state (callbacks vs. the POST worker thread).
        self._state_lock = threading.Lock()
        # Ensures only one HTTP request is in flight at a time.
        self._post_lock = threading.Lock()

        qos = (
            qos_profile_sensor_data
            if bool(self.get_parameter("use_sensor_qos").value)
            else 10
        )

        self.create_subscription(
            Float32,
            self.get_parameter("gas_concentration_topic").value,
            self._on_gas_concentration,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("gas_alert_topic").value,
            self._on_gas_alert,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("gas_type_topic").value,
            self._on_gas_type,
            qos,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("machine_temperatures_topic").value,
            self._on_machine_temperatures,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("thermal_alert_topic").value,
            self._on_thermal_alert,
            qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("robot_status_topic").value,
            self._on_robot_status,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("robot_mode_topic").value,
            self._on_robot_mode,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("moth_result_topic").value,
            self._on_moth_result,
            qos,
        )

        interval = float(self.get_parameter("post_interval_sec").value)
        self.create_timer(interval, self._post_latest_sensor_data)

        self.get_logger().info(
            f"LLM bridge ready: backend_url={self.backend_url}, "
            f"zones={len(self.zone_rectangles)}"
        )

    def _on_gas_concentration(self, msg: Float32):
        with self._state_lock:
            self.latest_gas_concentration = float(msg.data)

    def _on_gas_alert(self, msg: String):
        with self._state_lock:
            self.latest_gas_alert = msg.data.strip() or None

    def _on_gas_type(self, msg: String):
        with self._state_lock:
            self.latest_gas_type = self._normalize_gas_type(msg.data)

    def _on_machine_temperatures(self, msg: Float32MultiArray):
        with self._state_lock:
            self.latest_machine_temperatures = [float(value) for value in msg.data]

    def _on_thermal_alert(self, msg: String):
        with self._state_lock:
            self.latest_thermal_alert = msg.data.strip() or None

    def _on_odom(self, msg: Odometry):
        with self._state_lock:
            self.latest_odom_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )

    def _on_robot_status(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        with self._state_lock:
            # 같은 메시지 연속 중복 방지 (patrol이 재발행하는 경우)
            if self.status_events and self.status_events[-1]["text"] == text:
                return
            self.status_events.append(
                {
                    "time": _time.strftime("%H:%M:%S"),
                    "text": text,
                }
            )

    def _on_robot_mode(self, msg: String):
        with self._state_lock:
            self.latest_robot_mode = msg.data.strip() or None

    def _on_moth_result(self, msg: String):
        # 형식: "SOURCE_FOUND|NH3|85.3|x,y,z|DANGER"
        text = msg.data.strip()
        if "SOURCE_FOUND" not in text:
            return
        parts = text.split("|")
        try:
            peak = float(parts[2]) if len(parts) > 2 else None
        except ValueError:
            peak = None

        # Unity 좌표 → ROS 좌표 변환 + 구역 판정 (대시보드 '누출원: D구역 (x, y)' 표시용)
        zone = None
        ros_xy = None
        if len(parts) > 3:
            try:
                ux, uy, uz = [float(v) for v in parts[3].split(",")]
                rx, ry = uz, -ux
                ros_xy = f"({rx:.1f}, {ry:.1f})"
                zone = self._zone_name(rx, ry)
            except ValueError:
                pass

        with self._state_lock:
            self.latest_moth_result = {
                "gas_type": self._normalize_gas_type(parts[1]) if len(parts) > 1 else None,
                "peak_concentration": peak,
                "position_unity_xyz": parts[3] if len(parts) > 3 else None,
                "position_ros_xy": ros_xy,
                "zone": zone,
                "danger": (parts[4] == "DANGER") if len(parts) > 4 else None,
            }

    def _post_latest_sensor_data(self):
        # Build the payload under the lock, then hand the network I/O to a worker
        # thread so a slow / unreachable backend never blocks the executor (and
        # therefore subscription callbacks).
        with self._state_lock:
            if not self._has_any_sensor_data():
                return
            payload = self._build_payload()

        payload_json = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if payload_json == self._last_payload_json:
            return

        # Skip if a previous POST is still running (e.g. backend hanging).
        if not self._post_lock.acquire(blocking=False):
            self.get_logger().warn(
                "Previous POST still in flight; skipping this cycle.",
                throttle_duration_sec=5.0,
            )
            return

        worker = threading.Thread(
            target=self._send_payload, args=(payload_json,), daemon=True
        )
        worker.start()

    def _send_payload(self, payload_json: str):
        try:
            request = urllib.request.Request(
                self.backend_url,
                data=payload_json.encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(
                request, timeout=self.request_timeout_sec
            ) as response:
                status_code = getattr(response, "status", None)

            if status_code is not None and not (200 <= status_code < 300):
                self.get_logger().warn(
                    f"Backend returned non-2xx status={status_code}",
                    throttle_duration_sec=5.0,
                )
                return

            # Only treat a payload as "sent" once it actually succeeded, so a
            # failed POST is retried next cycle.
            self._last_payload_json = payload_json
            self.get_logger().info(
                f"Posted sensor data to LLM backend (status={status_code})"
            )
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            # Note: urlopen read timeouts raise socket.timeout (== TimeoutError),
            # which is NOT a URLError subclass, so it must be caught explicitly.
            self.get_logger().warn(
                f"Failed to post sensor data to {self.backend_url}: {exc}",
                throttle_duration_sec=5.0,
            )
        finally:
            self._post_lock.release()

    def _has_any_sensor_data(self) -> bool:
        return any(
            (
                self.latest_gas_concentration is not None,
                self.latest_gas_alert,
                self.latest_gas_type,
                self.latest_machine_temperatures,
                self.latest_thermal_alert,
                self.latest_odom_xy is not None,
                self.latest_robot_mode,
                self.latest_moth_result,
                len(self.status_events) > 0,
            )
        )

    def _build_payload(self) -> dict:
        gas_type = (
            self.latest_gas_type
            or self._infer_gas_type_from_alert(self.latest_gas_alert)
            or self.default_gas_type
        )

        return {
            "gas_type": gas_type,
            "gas_concentration": (
                float(self.latest_gas_concentration)
                if self.latest_gas_concentration is not None
                else 0.0
            ),
            "gas_alert": self.latest_gas_alert,
            "machine_temperatures": self.latest_machine_temperatures,
            "thermal_alert": self.latest_thermal_alert,
            "location": self._resolve_location(),
            # ── 리포트 생성용 사건 데이터 ──
            "robot_mode": self.latest_robot_mode,          # PATROL / GAS_TRACKING / EVACUATING
            "source_found": self.latest_moth_result,       # 누출원 특정 결과 (좌표·농도·위험여부)
            "status_events": list(self.status_events),     # 시간순 상황 보고 타임라인
        }

    def _zone_name(self, x_pos: float, y_pos: float) -> str | None:
        for zone in self.zone_rectangles:
            if (
                zone["x_min"] <= x_pos <= zone["x_max"]
                and zone["y_min"] <= y_pos <= zone["y_max"]
            ):
                return zone["name"]
        return None

    def _resolve_location(self) -> str:
        if self.latest_odom_xy is None:
            return self.default_location

        x_pos, y_pos = self.latest_odom_xy
        name = self._zone_name(x_pos, y_pos)
        if name:
            return name
        return f"map(x={x_pos:.2f}, y={y_pos:.2f})"

    def _load_zone_rectangles(self, raw_value: str) -> list[dict]:
        if not raw_value:
            return []

        try:
            data = json.loads(raw_value)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f"zone_rectangles JSON parse failed: {exc}. Zones disabled."
            )
            return []

        zones = []
        for item in data:
            try:
                zones.append(
                    {
                        "name": str(item["name"]),
                        "x_min": float(item["x_min"]),
                        "x_max": float(item["x_max"]),
                        "y_min": float(item["y_min"]),
                        "y_max": float(item["y_max"]),
                    }
                )
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f"Skipping invalid zone definition: {item}")

        return zones

    def _infer_gas_type_from_alert(self, alert_message: str | None) -> str | None:
        if not alert_message:
            return None

        uppercase_message = alert_message.upper()
        for alias, normalized in GAS_ALIASES.items():
            if alias in uppercase_message:
                return normalized

        match = re.search(r"\b(NH3|H2S|CH4|LNG)\b", uppercase_message)
        if match:
            return GAS_ALIASES.get(match.group(1), match.group(1))

        return None

    def _normalize_gas_type(self, value: str | None) -> str | None:
        if not value:
            return None

        normalized = GAS_ALIASES.get(value.strip().upper())
        if normalized:
            return normalized

        return value.strip()


def main(args=None):
    rclpy.init(args=args)
    node = LlmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
