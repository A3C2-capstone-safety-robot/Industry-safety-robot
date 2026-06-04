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
import re
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

# 구역 정의 - 좌표 경계는 예서님 맵 기준. 지영님 맵에 맞게 숫자 조정 필요.
ZONE_DEFINITIONS = {
    'A구역': lambda x, y: x < 0 and y < -5,
    'B구역': lambda x, y: x < 0 and y >= -5,
    'C구역': lambda x, y: 0 <= x < 21 and y < 0,
    'D구역': lambda x, y: 0 <= x < 21 and y >= 0,
    'E구역': lambda x, y: x >= 21,
}


def get_zone(x, y):
    for zone, cond in ZONE_DEFINITIONS.items():
        if cond(x, y):
            return zone
    return '알수없음'


class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')

        default_points = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'inspection_points.yaml'
        )
        self.declare_parameter('inspection_yaml', default_points)
        self.declare_parameter('dwell_time', 3.0)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('grid_resolution', 2.0)
        self.declare_parameter('free_pixel_thresh', 200)

        self.inspection_yaml = self.get_parameter('inspection_yaml').value
        self.dwell_time = float(self.get_parameter('dwell_time').value)

        # 과열 단계 임계값 (Unity MachineHeat 와 동기화)
        self.caution_thres = 80.0
        self.warning_thres = 100.0
        self.danger_thres = 120.0
        self.max_temp = 180.0
        self.overheat_cooldown = 15.0
        self._overheat_reported = {}
        self.gas_tracking = False
        self._gas_danger = False
        self._gas_start_time = None
        self.gas_timeout = 45.0  # 누출원 못 찾으면 순찰 재개(초)

        self.patrol_active = True
        self.evacuating = False
        self.current_x = 0.0
        self.current_y = 0.0
        self._current_goal_handle = None
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=cb)
        self.create_subscription(String, '/thermal_alerts', self.thermal_callback, 10, callback_group=cb)
        self.create_subscription(String, '/gas_alert', self.gas_callback, 10, callback_group=cb)
        self.create_subscription(String, '/moth_search/result', self.moth_result_callback, 10, callback_group=cb)

        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self._mode_pub = self.create_publisher(String, '/robot_mode', 10)

        self.inspection_points = self.load_inspection_points()
        self.get_logger().info('점검 지점 %d개 로드 완료' % len(self.inspection_points))

    def load_inspection_points(self):
        if not os.path.exists(self.inspection_yaml):
            self.get_logger().error('점검 지점 파일을 찾을 수 없음: %s' % self.inspection_yaml)
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
                self.get_logger().warn('잘못된 점검 지점 건너뜀: %s' % item)
        return points

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def thermal_callback(self, msg):
        text = msg.data
        m = re.search(r'\[([^\]]+)\]\s*([\d.]+)\s*C', text)
        if not m:
            return

        machine = m.group(1)
        temp = float(m.group(2))
        if temp < self.caution_thres:
            return

        now = time.time()
        if now - self._overheat_reported.get(machine, 0.0) < self.overheat_cooldown:
            return
        self._overheat_reported[machine] = now

        if temp >= self.max_temp:
            level, need_evac = '최대(즉시대피)', True
        elif temp >= self.danger_thres:
            level, need_evac = '위험', False
        elif temp >= self.warning_thres:
            level, need_evac = '경고', False
        else:
            level, need_evac = '주의', False

        zone = get_zone(self.current_x, self.current_y)
        guide = text.split('-', 1)[1].strip() if '-' in text else ''
        report = ('[과열] %s %.1f°C | 단계:%s | 구역:%s | 위치:(%.1f,%.1f) | 조치:%s'
                  % (machine, temp, level, zone, self.current_x, self.current_y, guide))

        if need_evac:
            with self._lock:
                do_evac = not self.evacuating
                if do_evac:
                    self.evacuating = True
                    self.patrol_active = False
            nearest = self.find_nearest_door()
            report += ' | 대피필요 → 최근접출구:%s' % nearest['name']
            self.publish_status(report)
            self.get_logger().error('🔥 %s' % report)
            if do_evac:
                threading.Thread(target=self.evacuate, args=(nearest,), daemon=True).start()
        else:
            self.publish_status(report)
            self.get_logger().warn('🔥 %s' % report)

    def gas_callback(self, msg):
        # 형식: "[DANGER] NH3 감지: 52.3 ppm @ 위치(x, y, z)"
        text = msg.data
        if '감지' not in text:
            return
        is_danger = 'DANGER' in text.upper() or '[위험]' in text

        # ── 추적 중: 농도가 위험 수준으로 올라가면 즉시 위험 리포트 (1회, 추적은 계속) ──
        if self.gas_tracking:
            if is_danger and not self._gas_danger:
                self._gas_danger = True
                gtype = self._search(r'\]\s*(\S+)\s*감지', text, '미상')
                conc = self._search(r'([\d.]+)\s*ppm', text, '?')
                zone = get_zone(self.current_x, self.current_y)
                report = ('[가스위험] 종류:%s | 농도:%sppm | 구역:%s | '
                          '위험 수준 도달 — 인근 인원 대피 필요 (로봇은 누출원 추적 계속)'
                          % (gtype, conc, zone))
                self.publish_status(report)
                self.get_logger().error('🚨 %s' % report)
            return

        with self._lock:
            if self.evacuating or self.gas_tracking:
                return
            self.gas_tracking = True
            self.patrol_active = False
            self._gas_danger = False   # 위험 여부는 추적 중 실시간으로 판정
            self._gas_start_time = time.time()

        gtype = self._search(r'\]\s*(\S+)\s*감지', text, '미상')
        conc = self._search(r'([\d.]+)\s*ppm', text, '?')
        zone = get_zone(self.current_x, self.current_y)
        report1 = ('[가스감지] 종류:%s | 농도:%sppm | 구역:%s | 누출원 추적 시작'
                   % (gtype, conc, zone))
        self.publish_status(report1)
        self.get_logger().warn('🟡 %s' % report1)
        # 추적 모드로 전환 → Unity 코디네이터가 MothSearch 켬, 순찰은 멈춤
        self.publish_mode('GAS_TRACKING')

    def moth_result_callback(self, msg):
        # 형식: "SOURCE_FOUND|NH3|85.3|x,y,z"
        if not self.gas_tracking:
            return
        text = msg.data
        if 'SOURCE_FOUND' not in text:
            return
        parts = text.split('|')
        gtype = parts[1] if len(parts) > 1 else '미상'
        conc = parts[2] if len(parts) > 2 else '?'
        coord = parts[3] if len(parts) > 3 else '?'
        zone = '미상'
        coord_str = coord
        try:
            ux, uy, uz = [float(v) for v in coord.split(',')]
            rx, ry = uz, -ux  # Unity → ROS 좌표
            zone = get_zone(rx, ry)
            coord_str = '(%.1f, %.1f)' % (rx, ry)
        except (ValueError, IndexError):
            pass

        need_evac = self._gas_danger
        report2 = ('[누출원발견] 종류:%s | 농도:%sppm | 좌표:%s | 구역:%s | 대피:%s'
                   % (gtype, conc, coord_str, zone, '필요' if need_evac else '불필요'))
        self.publish_status(report2)
        self.get_logger().warn('🔴 %s' % report2)

        self.gas_tracking = False
        self._gas_start_time = None
        if need_evac:
            with self._lock:
                do_evac = not self.evacuating
                if do_evac:
                    self.evacuating = True
                    self.patrol_active = False
            self.publish_mode('EVACUATING')
            nearest = self.find_nearest_door()
            if do_evac:
                threading.Thread(target=self.evacuate, args=(nearest,), daemon=True).start()
        else:
            self.patrol_active = True
            self.publish_mode('PATROL')

    def publish_mode(self, mode):
        m = String()
        m.data = mode
        self._mode_pub.publish(m)

    def _search(self, pattern, text, default):
        mt = re.search(pattern, text)
        return mt.group(1) if mt else default

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def find_nearest_door(self):
        nearest = None
        min_dist = float('inf')
        for door in DOOR_POSITIONS:
            dist = math.hypot(door['x'] - self.current_x, door['y'] - self.current_y)
            if dist < min_dist:
                min_dist = dist
                nearest = door
        return nearest

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

    def evacuate(self, door):
        time.sleep(2.0)
        self.get_logger().warn('🚶 대피 → %s' % door['name'])
        success = self.go_to_pose(door['x'], door['y'], 0.0, abort_check=lambda: False)
        if success:
            self.get_logger().warn('✅ 대피 완료! %s 도착. 30초 후 순찰 재개...' % door['name'])
            self.publish_status('[대피완료] %s 도착. 순찰 재개 대기' % door['name'])
            time.sleep(30)
        else:
            self.get_logger().warn('대피 이동 실패')

        with self._lock:
            self.evacuating = False
            self.patrol_active = True
        self._overheat_reported.clear()
        self.gas_tracking = False
        self.publish_mode('PATROL')
        self.get_logger().info('🔄 순찰 재개!')

    def _interrupted(self):
        return self.evacuating or not self.patrol_active

    def _check_gas_timeout(self):
        # 가스 추적이 너무 오래 지속(누출원 못 찾음/해결됨)되면 순찰 재개
        if (self.gas_tracking and not self.evacuating
                and self._gas_start_time is not None
                and time.time() - self._gas_start_time > self.gas_timeout):
            self.get_logger().warn('⏱ 가스 추적 타임아웃 — 누출원 못 찾음/해결됨, 순찰 재개')
            self.publish_status('[가스] 추적 타임아웃 또는 해결 — 순찰 재개')
            self.gas_tracking = False
            self._gas_start_time = None
            self.patrol_active = True
            self.publish_mode('PATROL')

    def run_patrol(self):
        self.get_logger().info('NavigateToPose 액션 서버 대기 중...')
        self._nav_client.wait_for_server()
        self.get_logger().info('액션 서버 연결됨! 순찰 시작!')

        if not self.inspection_points:
            self.get_logger().error('점검 지점이 없습니다. 종료.')
            return

        total = len(self.inspection_points)
        patrol_count = 0

        while rclpy.ok():
            if self._interrupted():
                self._check_gas_timeout()
                time.sleep(1.0)
                continue

            patrol_count += 1
            self.get_logger().info('===== 순찰 %d회차 시작 (%d개 지점) =====' % (patrol_count, total))
            start_time = time.time()
            round_success = 0

            for idx, (name, x, y, yaw) in enumerate(self.inspection_points):
                if self._interrupted():
                    break

                self.get_logger().info('[%d회차] %d/%d → %s (%.2f, %.2f) 이동 중...'
                                       % (patrol_count, idx + 1, total, name, x, y))
                success = self.go_to_pose(x, y, yaw, abort_check=self._interrupted)
                if self._interrupted():
                    self.get_logger().warn('🚨 순찰 중단 (대피)')
                    break

                if success:
                    round_success += 1
                    self.get_logger().info('  ✅ %s 도착 — 점검 중 (%.0f초)' % (name, self.dwell_time))
                    t0 = time.time()
                    while time.time() - t0 < self.dwell_time:
                        if self._interrupted():
                            break
                        time.sleep(0.2)
                else:
                    # 목표 거부/실패 → 폭주 방지 백오프
                    self.get_logger().warn('  ⚠ %s 도달 실패 — 2초 후 다음 지점' % name)
                    time.sleep(2.0)

            if self._interrupted():
                continue

            if round_success == 0:
                self.get_logger().error(
                    '⚠️ 한 지점도 도달 못함 — Nav2/Unity 가 아직 준비 안 됨? '
                    '(Unity Play 했는지, /odom·/scan 흐르는지 확인) 8초 대기.')
                time.sleep(8.0)
            else:
                elapsed = time.time() - start_time
                self.get_logger().info('✅ 순찰 %d회차 완료 (소요: %d분 %d초)'
                                       % (patrol_count, int(elapsed // 60), int(elapsed % 60)))
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
