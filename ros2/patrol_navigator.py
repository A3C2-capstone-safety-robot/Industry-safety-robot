#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import yaml
import math
import os
import time
import threading


DOOR_POSITIONS = [
    {'name': 'exit_door_01_A', 'x': -8.55,  'y': 16.0},
    {'name': 'exit_door_01_B', 'x': 15.0,   'y': -15.0},
    {'name': 'exit_door_01_C', 'x': 15.0,   'y': 10.39},
    {'name': 'exit_door_02_A', 'x': -16.0,  'y': 16.5},
    {'name': 'exit_door_02_B', 'x': -3.91,  'y': -15.0},
    {'name': 'exit_door_02_C', 'x': 14.5,   'y': 17.0},
]


class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')

        # 점검 지점 파일: 이 스크립트와 같은 폴더의 inspection_points.yaml
        default_points = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'inspection_points.yaml'
        )
        self.declare_parameter('inspection_yaml', default_points)
        # 각 기계 앞에서 멈춰 점검하는 시간(초)
        self.declare_parameter('dwell_time', 3.0)
        # (구버전 호환용 — 지금은 안 쓰지만, 기존 실행 명령이 깨지지 않게 남겨둠)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('grid_resolution', 2.0)
        self.declare_parameter('free_pixel_thresh', 200)

        self.inspection_yaml = self.get_parameter('inspection_yaml').value
        self.dwell_time = float(self.get_parameter('dwell_time').value)

        self.patrol_active = True
        self.evacuating = False
        self.current_x = 0.0
        self.current_y = 0.0
        self._current_goal_handle = None
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=cb)
        self.create_subscription(String, '/thermal_alerts', self.alert_callback, 10, callback_group=cb)
        self.create_subscription(String, '/gas_alert', self.alert_callback, 10, callback_group=cb)

        self.inspection_points = self.load_inspection_points()
        self.get_logger().info(f'점검 지점 {len(self.inspection_points)}개 로드 완료')
        for name, x, y, _ in self.inspection_points:
            self.get_logger().info(f'  - {name} ({x:.2f}, {y:.2f})')

    # ────────────────────────────────────────────────
    #  점검 지점 로드
    # ────────────────────────────────────────────────
    def load_inspection_points(self):
        if not os.path.exists(self.inspection_yaml):
            self.get_logger().error(f'점검 지점 파일을 찾을 수 없음: {self.inspection_yaml}')
            return []

        with open(self.inspection_yaml, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        points = []
        for item in (data.get('inspection_points') or []):
            try:
                points.append((
                    str(item['name']),
                    float(item['x']),
                    float(item['y']),
                    float(item.get('yaw', 0.0)),
                ))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f'잘못된 점검 지점 건너뜀: {item}')
        return points

    # ────────────────────────────────────────────────
    #  콜백
    # ────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def alert_callback(self, msg):
        with self._lock:
            if self.evacuating:
                return
            text = msg.data
            is_danger = '[위험]' in text or '[경고]' in text or '가스' in text or 'gas' in text.lower()
            if not is_danger:
                return
            self.evacuating = True
            self.patrol_active = False

        self.get_logger().warn(f'🚨 위험 경보 수신! "{text[:50]}"')
        nearest = self.find_nearest_door()
        self.get_logger().warn(f'🚪 가장 가까운 문: {nearest["name"]} → 즉시 대피!')

        evac_thread = threading.Thread(target=self.evacuate, args=(nearest,), daemon=True)
        evac_thread.start()

    def find_nearest_door(self):
        nearest = None
        min_dist = float('inf')
        for door in DOOR_POSITIONS:
            dist = math.hypot(door['x'] - self.current_x, door['y'] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                nearest = door
        return nearest

    # ────────────────────────────────────────────────
    #  목표 이동 (NavigateToPose) — 도착/실패/대피인터럽트까지 대기
    # ────────────────────────────────────────────────
    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def go_to_pose(self, x, y, yaw, abort_check=None):
        """목표까지 이동. 성공 True / 실패·중단 False. abort_check()가 True면 취소."""
        self._nav_client.wait_for_server()

        done = threading.Event()
        status = [None]

        def goal_response_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warn('목표 거부됨')
                done.set()
                return
            self._current_goal_handle = gh

            def result_cb(rf):
                status[0] = rf.result().status
                done.set()

            gh.get_result_async().add_done_callback(result_cb)

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(goal_response_cb)

        while not done.is_set():
            if abort_check is not None and abort_check():
                if self._current_goal_handle is not None:
                    self._current_goal_handle.cancel_goal_async()
                return False
            time.sleep(0.2)

        return status[0] == GoalStatus.STATUS_SUCCEEDED

    # ────────────────────────────────────────────────
    #  대피
    # ────────────────────────────────────────────────
    def evacuate(self, door):
        time.sleep(2.0)
        self.get_logger().warn(f'🚶 대피 → {door["name"]}')
        success = self.go_to_pose(door['x'], door['y'], 0.0, abort_check=lambda: False)
        if success:
            self.get_logger().warn(f'✅ 대피 완료! {door["name"]} 도착. 30초 후 순찰 재개...')
            time.sleep(30)
        else:
            self.get_logger().warn('대피 이동 실패')

        with self._lock:
            self.evacuating = False
            self.patrol_active = True
        self.get_logger().info('🔄 순찰 재개!')

    # ────────────────────────────────────────────────
    #  순찰 루프 — 점검 지점을 순서대로 방문 + 각 지점에서 멈춰 점검
    # ────────────────────────────────────────────────
    def _interrupted(self):
        return self.evacuating or not self.patrol_active

    def run_patrol(self):
        self.get_logger().info('NavigateToPose 액션 서버 대기 중...')
        self._nav_client.wait_for_server()
        self.get_logger().info('액션 서버 연결됨! 순찰 시작!')

        if not self.inspection_points:
            self.get_logger().error('점검 지점이 없습니다. inspection_points.yaml 확인 필요. 종료.')
            return

        total = len(self.inspection_points)
        patrol_count = 0

        while rclpy.ok():
            if self._interrupted():
                time.sleep(1.0)
                continue

            patrol_count += 1
            self.get_logger().info(f'===== 순찰 {patrol_count}회차 시작 ({total}개 지점) =====')
            start_time = time.time()

            for idx, (name, x, y, yaw) in enumerate(self.inspection_points):
                if self._interrupted():
                    break

                self.get_logger().info(
                    f'[{patrol_count}회차] {idx + 1}/{total} → {name} ({x:.2f}, {y:.2f}) 이동 중...'
                )
                success = self.go_to_pose(x, y, yaw, abort_check=self._interrupted)

                if self._interrupted():
                    self.get_logger().warn('🚨 순찰 중단 (대피)')
                    break

                if success:
                    self.get_logger().info(f'  ✅ {name} 도착 — 점검 중 ({self.dwell_time:.0f}초)')
                    t0 = time.time()
                    while time.time() - t0 < self.dwell_time:
                        if self._interrupted():
                            break
                        time.sleep(0.2)
                else:
                    self.get_logger().warn(f'  ⚠ {name} 도달 실패 — 다음 지점으로')

            if not self._interrupted():
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f'✅ 순찰 {patrol_count}회차 완료 (소요: {int(elapsed // 60)}분 {int(elapsed % 60)}초)'
                )
                time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()

    patrol_thread = threading.Thread(target=node.run_patrol, daemon=True)
    patrol_thread.start()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
