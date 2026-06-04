#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import yaml
import math
import os
import re
import time
import threading


# 기본 출구 좌표 — exit_doors.yaml(teach_doors.py로 티칭)이 있으면 그쪽이 우선
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
        self.evac_thres = 150.0   # 대피 발동 온도 — Unity maxTemp(180)에 닿기 전에 대피
        self.max_temp = 180.0
        self.overheat_cooldown = 15.0
        self._overheat_reported = {}
        self.gas_tracking = False
        self._gas_danger = False
        self._gas_start_time = None
        self._last_gas_seen = None
        self.gas_lost_timeout = 30.0   # 이 시간 동안 가스 미검출 → 상황 종료로 판단(초)
                                       # (Cast/복귀 중 농도가 임계 밑으로 떨어져 알림이 끊길 수 있어 여유 필요)
        self._last_source_handled = 0.0  # SOURCE_FOUND 중복 처리 방지 (Unity가 2초 주기 재발행)
        self.gas_timeout = 180.0       # 하드캡: 이 시간 넘게 못 찾으면 실패 보고 후 복귀(초)

        self.patrol_active = True
        self.evacuating = False
        self._leak_xy = None           # 마지막 확인된 누출원 위치 (ROS 좌표) — 대피 경로 회피용
        self._resume_idx = 0           # 중단된 순찰을 이어서 할 지점 인덱스
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
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # 스턱 탈출용 직접 제어

        self.inspection_points = self.load_inspection_points()
        self.get_logger().info('점검 지점 %d개 로드 완료' % len(self.inspection_points))

        self.doors = self.load_exit_doors()
        self.get_logger().info('대피 출구 %d개 로드 완료' % len(self.doors))

    def load_exit_doors(self):
        """teach_doors.py로 티칭한 exit_doors.yaml 우선, 없으면 내장 기본 좌표."""
        doors_yaml = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'exit_doors.yaml')
        if not os.path.exists(doors_yaml):
            self.get_logger().warn(
                'exit_doors.yaml 없음 — 내장 기본 출구 좌표 사용 '
                '(맵에 안 맞으면 teach_doors.py로 티칭하세요)')
            return list(DOOR_POSITIONS)
        with open(doors_yaml, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        doors = []
        for item in (data.get('exit_doors') or []):
            try:
                doors.append({
                    'name': str(item['name']),
                    'x': float(item['x']),
                    'y': float(item['y']),
                })
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn('잘못된 출구 항목 건너뜀: %s' % item)
        if not doors:
            self.get_logger().warn('exit_doors.yaml이 비어 있음 — 내장 기본 좌표 사용')
            return list(DOOR_POSITIONS)
        return doors

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

        if temp >= self.evac_thres:
            level, need_evac = '심각(즉시대피)', True
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
                    self.gas_tracking = False   # 가스 추적 중이었다면 중단
            # EVACUATING 모드 발행 → Unity MissionCoordinator가 MothSearch를 꺼서
            # 대피(Nav2)와 가스추적이 /cmd_vel을 동시에 쏘는 충돌 방지
            if do_evac:
                self.publish_mode('EVACUATING')
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
            self._last_gas_seen = time.time()  # 가스가 아직 잡히고 있음
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
            self._last_gas_seen = time.time()

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
        # 형식: "SOURCE_FOUND|NH3|85.3|x,y,z|DANGER"
        text = msg.data
        if 'SOURCE_FOUND' not in text:
            return
        # ★ gas_tracking 플래그로 거르지 않음 — 추적 중 가스 알림이 잠시 끊겨
        #   타임아웃으로 플래그가 풀린 뒤 결과가 도착해도 레포트/대피는 수행해야 함.
        #   중복(재발행)은 쿨다운으로 차단.
        now = time.time()
        if now - self._last_source_handled < 15.0:
            return
        if self.evacuating:
            return
        self._last_source_handled = now

        parts = text.split('|')
        gtype = parts[1] if len(parts) > 1 else '미상'
        conc = parts[2] if len(parts) > 2 else '?'
        coord = parts[3] if len(parts) > 3 else '?'
        danger_flag = parts[4] if len(parts) > 4 else ''
        zone = '미상'
        coord_str = coord
        try:
            ux, uy, uz = [float(v) for v in coord.split(',')]
            rx, ry = uz, -ux  # Unity → ROS 좌표
            zone = get_zone(rx, ry)
            coord_str = '(%.1f, %.1f)' % (rx, ry)
            self._leak_xy = (rx, ry)   # 대피 출구 선택 시 이 지점 근처 경로 회피
        except (ValueError, IndexError):
            pass

        # 추적 중 DANGER 알림을 받았거나, Unity가 가스별 위험 기준 초과로 판정했으면 대피
        need_evac = self._gas_danger or danger_flag == 'DANGER'
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
        for door in self.doors:
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

    def go_to_pose(self, x, y, yaw, abort_check=None, timeout=None):
        self._nav_client.wait_for_server()
        done = threading.Event()
        status = [None]
        start_time = time.time()

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
            # 타임아웃 — Nav2가 막힌 채 무한 재계획하면 여기서 끊고 실패 처리
            if timeout is not None and time.time() - start_time > timeout:
                if self._current_goal_handle is not None:
                    self._current_goal_handle.cancel_goal_async()
                self.get_logger().warn('⏱ 이동 %d초 초과 — 목표 취소' % int(timeout))
                return False
            time.sleep(0.2)

        return status[0] == GoalStatus.STATUS_SUCCEEDED

    # 누출원 근처(반경 m)를 지나는 출구 경로에 줄 가산 비용
    LEAK_AVOID_RADIUS = 3.0
    LEAK_PENALTY = 100.0

    def _door_cost(self, d):
        """출구 우선순위 = 거리 + (직선 경로가 누출원 근처를 지나면 페널티).
        Nav2 실경로는 직선이 아니지만, '누출원 방향 출구를 후순위로' 미는
        휴리스틱으로는 충분. 누출원 미확인 시(_leak_xy None) 순수 거리순."""
        dist = math.hypot(d['x'] - self.current_x, d['y'] - self.current_y)
        if self._leak_xy is not None:
            seg_dist = self._seg_point_dist(
                self.current_x, self.current_y, d['x'], d['y'],
                self._leak_xy[0], self._leak_xy[1])
            if seg_dist < self.LEAK_AVOID_RADIUS:
                dist += self.LEAK_PENALTY
        return dist

    @staticmethod
    def _seg_point_dist(x1, y1, x2, y2, px, py):
        """선분 (x1,y1)-(x2,y2)와 점 (px,py) 사이 최단 거리"""
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-9:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

    def evacuate(self, door):
        time.sleep(2.0)
        # 가까운 순서대로 모든 출구 시도 — 단, 누출원 근처를 지나는 출구는 후순위
        doors = sorted(self.doors, key=self._door_cost)
        if self._leak_xy is not None:
            self.get_logger().warn('대피 출구 정렬: 누출원(%.1f, %.1f) 회피 반영'
                                   % self._leak_xy)
        success = False
        for attempt in range(2):                  # 전 출구 실패 시 1회 재시도
            for d in doors:
                self.get_logger().warn('🚶 대피 → %s' % d['name'])
                if self.go_to_pose(d['x'], d['y'], 0.0,
                                   abort_check=lambda: False, timeout=90.0):
                    success = True
                    self.get_logger().warn('✅ 대피 완료! %s 도착. 30초 후 순찰 재개...' % d['name'])
                    self.publish_status('[대피완료] %s 도착. 순찰 재개 대기' % d['name'])
                    time.sleep(30)
                    break
                self.get_logger().warn('⚠ %s 경로 막힘/실패 — 다음 출구 시도' % d['name'])
                self.publish_status('[대피실패] %s 도달 불가 — 다음 출구 시도' % d['name'])
            if success:
                break
            if attempt == 0:
                # 전 출구 즉시 실패 = 로봇이 장애물(과열 기계 등)에 붙어 있어
                # 출발점이 inflation 영역 안 → 플래너가 시작조차 못 하는 경우.
                # 살짝 후진해서 빠져나온 뒤 재시도 (왔던 길이라 후진은 안전).
                self.get_logger().warn('전 출구 실패 — 후진으로 장애물 이탈 후 재시도')
                self._nudge_backward(2.0)
                time.sleep(3.0)

        if not success:
            self.get_logger().error('❌ 모든 출구 도달 실패 — 현 위치 대기 후 순찰 복귀')
            self.publish_status('[대피불가] 모든 출구 도달 실패 — 수동 확인 필요')

        with self._lock:
            self.evacuating = False
            self.patrol_active = True
        self._overheat_reported.clear()
        self.gas_tracking = False
        self.publish_mode('PATROL')
        self.get_logger().info('🔄 순찰 재개!')

    def _nudge_backward(self, duration=2.0):
        """/cmd_vel 직접 발행으로 천천히 후진 — inflation 영역 탈출용."""
        twist = Twist()
        twist.linear.x = -0.15
        end = time.time() + duration
        while time.time() < end:
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._cmd_pub.publish(Twist())  # 정지

    def _interrupted(self):
        return self.evacuating or not self.patrol_active

    def _check_gas_timeout(self):
        if not self.gas_tracking or self.evacuating:
            return
        now = time.time()

        # 1) 가스 소실: 일정 시간 가스 알림이 없으면 누출 해소/소실로 판단
        if (self._last_gas_seen is not None
                and now - self._last_gas_seen > self.gas_lost_timeout):
            self.get_logger().warn('💨 가스 미검출 %d초 — 누출 해소/소실로 판단, 순찰 재개'
                                   % int(self.gas_lost_timeout))
            self.publish_status('[가스종료] 가스 미검출 %d초 — 누출 해소/소실로 판단. 순찰 재개'
                                % int(self.gas_lost_timeout))
            self._end_gas_tracking()
            return

        # 2) 하드캡: 가스는 계속 있는데 오래 못 찾으면 실패 보고 후 복귀
        if (self._gas_start_time is not None
                and now - self._gas_start_time > self.gas_timeout):
            self.get_logger().warn('⏱ 추적 %d초 초과 — 누출원 특정 실패, 수동 점검 필요'
                                   % int(self.gas_timeout))
            self.publish_status('[가스미특정] 추적 %d초 초과 — 누출원 특정 실패, 수동 점검 필요. 순찰 재개'
                                % int(self.gas_timeout))
            self._end_gas_tracking()

    def _end_gas_tracking(self):
        self.gas_tracking = False
        self._gas_start_time = None
        self._last_gas_seen = None
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
            start_time = time.time()
            round_success = 0

            # 대피/추적으로 중단됐던 지점부터 이어서 (한 바퀴 끝나면 0으로 리셋)
            start_idx = self._resume_idx
            self._resume_idx = 0
            if start_idx > 0:
                self.get_logger().info('===== 순찰 %d회차 — %d/%d번 지점부터 이어서 ====='
                                       % (patrol_count, start_idx + 1, total))
            else:
                self.get_logger().info('===== 순찰 %d회차 시작 (%d개 지점) =====' % (patrol_count, total))

            for idx in range(start_idx, total):
                name, x, y, yaw = self.inspection_points[idx]
                if self._interrupted():
                    self._resume_idx = idx
                    break

                self.get_logger().info('[%d회차] %d/%d → %s (%.2f, %.2f) 이동 중...'
                                       % (patrol_count, idx + 1, total, name, x, y))
                success = self.go_to_pose(x, y, yaw, abort_check=self._interrupted)
                if self._interrupted():
                    self._resume_idx = idx   # 이 지점부터 재개
                    self.get_logger().warn('🚨 순찰 중단 (대피) — 재개 시 %d번 지점부터' % (idx + 1))
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
# EOF
