#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import numpy as np
from PIL import Image
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

        # 기본 맵 경로: 이 스크립트와 같은 위치의 maps/factory_map.yaml 을 자동으로 찾음
        # (사용자명에 묶이지 않음. 필요하면 -p map_yaml:=... 로 덮어쓸 수 있음)
        default_map_yaml = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'maps', 'factory_map.yaml'
        )
        self.declare_parameter('map_yaml', default_map_yaml)
        self.declare_parameter('grid_resolution', 2.0)
        self.declare_parameter('free_pixel_thresh', 200)

        self.map_yaml = self.get_parameter('map_yaml').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.free_pixel_thresh = self.get_parameter('free_pixel_thresh').value

        self.patrol_active = True
        self.evacuating = False
        self.current_x = 0.0
        self.current_y = 0.0
        self._current_patrol_count = 0
        self._last_logged_waypoint = -1
        self._patrol_start_time = None
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints', callback_group=cb)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=cb)
        self.create_subscription(String, '/thermal_alerts', self.alert_callback, 10, callback_group=cb)
        self.create_subscription(String, '/gas_alert', self.alert_callback, 10, callback_group=cb)

        self.waypoints = self.generate_patrol_waypoints()
        self.get_logger().info(f'순찰 웨이포인트 {len(self.waypoints)}개 생성 완료')

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
        self.get_logger().warn(f'🚪 가장 가까운 문: {nearest["name"]} ({nearest["x"]:.2f}, {nearest["y"]:.2f}) → 즉시 대피!')

        evac_thread = threading.Thread(target=self.evacuate, args=(nearest,), daemon=True)
        evac_thread.start()

    def find_nearest_door(self):
        nearest = None
        min_dist = float('inf')
        for door in DOOR_POSITIONS:
            dist = math.sqrt((door['x'] - self.current_x) ** 2 + (door['y'] - self.current_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                nearest = door
        self.get_logger().info(f'현재 위치: ({self.current_x:.2f}, {self.current_y:.2f}), 최근접 문까지: {min_dist:.2f}m')
        return nearest

    def evacuate(self, door):
        time.sleep(4.0)

        for attempt in range(10):
            self.get_logger().warn(f'🚶 대피 시도 {attempt + 1}/3 → {door["name"]}')

            evac_done = threading.Event()
            evac_succeeded = [False]

            def goal_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn('대피 목표 거부됨')
                    evac_done.set()
                    return

                def result_cb(result_future):
                    result = result_future.result()
                    evac_succeeded[0] = (result.status == GoalStatus.STATUS_SUCCEEDED)
                    evac_done.set()

                goal_handle.get_result_async().add_done_callback(result_cb)

            self._nav_client.wait_for_server()
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(door['x'])
            goal.pose.pose.position.y = float(door['y'])
            goal.pose.pose.orientation.w = 1.0

            send_future = self._nav_client.send_goal_async(goal)
            send_future.add_done_callback(goal_response_cb)

            evac_done.wait(timeout=120.0)

            if evac_succeeded[0]:
                self.get_logger().warn(f'✅ 대피 완료! {door["name"]} 도착! 30초 후 순찰 재개...')
                time.sleep(30)
                break
            else:
                self.get_logger().warn(f'대피 실패 (시도 {attempt + 1}/3), 3초 후 재시도...')
                time.sleep(3)

        with self._lock:
            self.evacuating = False
            self.patrol_active = True
        self.get_logger().info('🔄 순찰 재개!')

    def load_map(self):
        with open(self.map_yaml, 'r') as f:
            map_data = yaml.safe_load(f)
        map_dir = os.path.dirname(os.path.abspath(self.map_yaml))
        pgm_path = os.path.join(map_dir, map_data['image'])
        img = Image.open(pgm_path).convert('L')
        return np.array(img), map_data

    def generate_patrol_waypoints(self):
        map_array, map_data = self.load_map()
        resolution = map_data['resolution']
        origin = map_data['origin']
        negate = map_data.get('negate', 0)
        height, width = map_array.shape
        grid_px = max(1, int(self.grid_resolution / resolution))

        raw_points = []
        rows = list(range(grid_px, height - grid_px, grid_px))
        for row_idx, row in enumerate(rows):
            cols = range(grid_px, width - grid_px, grid_px)
            if row_idx % 2 == 1:
                cols = reversed(list(cols))
            for col in cols:
                pixel_val = int(map_array[row, col])
                occupancy = pixel_val / 255.0 if negate else 1.0 - pixel_val / 255.0
                if occupancy < 0.3:
                    x = origin[0] + col * resolution
                    y = origin[1] + (height - row) * resolution
                    raw_points.append((x, y))

        waypoints = []
        for i, (x, y) in enumerate(raw_points):
            if i + 1 < len(raw_points):
                nx, ny = raw_points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            waypoints.append((x, y, yaw))
        return waypoints

    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        total = len(self.waypoints)
        if current != self._last_logged_waypoint:
            self._last_logged_waypoint = current
            if current < total:
                x, y, _ = self.waypoints[current]
                progress = (current / total) * 100
                elapsed = time.time() - self._patrol_start_time
                elapsed_str = f'{int(elapsed // 60)}분 {int(elapsed % 60)}초'
                if progress > 0:
                    remaining = elapsed / (progress / 100) - elapsed
                    remaining_str = f'{int(remaining // 60)}분 {int(remaining % 60)}초'
                else:
                    remaining_str = '계산 중'
                self.get_logger().info(
                    f'[{self._current_patrol_count}회차] 웨이포인트 {current}/{total} 도달 '
                    f'→ 다음: ({x:.2f}, {y:.2f}) '
                    f'[{progress:.1f}% | 경과: {elapsed_str} | 남은: {remaining_str}]'
                )

    def run_patrol(self):
        self.get_logger().info('FollowWaypoints 액션 서버 대기 중...')
        self._waypoint_client.wait_for_server()
        self.get_logger().info('액션 서버 연결됨! 순찰 시작!')

        patrol_count = 0
        while rclpy.ok():
            if not self.patrol_active or self.evacuating:
                time.sleep(1.0)
                continue

            patrol_count += 1
            self._current_patrol_count = patrol_count
            self._last_logged_waypoint = -1
            self._patrol_start_time = time.time()
            self.get_logger().info(f'===== 순찰 {patrol_count}회차 시작 ({len(self.waypoints)}개 웨이포인트) =====')

            patrol_done = threading.Event()
            patrol_result = [None]

            def goal_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn('순찰 목표 거절됨')
                    patrol_result[0] = 'rejected'
                    patrol_done.set()
                    return

                def result_cb(result_future):
                    patrol_result[0] = result_future.result()
                    patrol_done.set()

                goal_handle.get_result_async().add_done_callback(result_cb)
                self._current_goal_handle = goal_handle

            goal = FollowWaypoints.Goal()
            goal.poses = [self.make_pose(x, y, yaw) for x, y, yaw in self.waypoints]

            future = self._waypoint_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            future.add_done_callback(goal_response_cb)

            while not patrol_done.is_set():
                if self.evacuating:
                    if hasattr(self, '_current_goal_handle'):
                        self._current_goal_handle.cancel_goal_async()
                    self.get_logger().warn('🚨 대피로 인해 순찰 중단!')
                    break
                time.sleep(0.5)

            if self.evacuating:
                while self.evacuating:
                    time.sleep(1.0)
                continue

            patrol_done.wait(timeout=5.0)
            result = patrol_result[0]

            total_time = time.time() - self._patrol_start_time
            total_str = f'{int(total_time // 60)}분 {int(total_time % 60)}초'

            if result == 'rejected':
                self.get_logger().warn('목표 거절됨, 5초 후 재시도...')
                time.sleep(5)
            elif result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'✅ 순찰 {patrol_count}회차 완료! (소요: {total_str})')
                time.sleep(3)
            else:
                if result is not None:
                    missed = result.result.missed_waypoints
                    self.get_logger().warn(
                        f'순찰 {patrol_count}회차 종료: '
                        f'{len(self.waypoints) - len(missed)}/{len(self.waypoints)}개 도달 (소요: {total_str})'
                    )
                time.sleep(2)


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
