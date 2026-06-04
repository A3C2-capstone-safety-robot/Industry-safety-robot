#!/usr/bin/env python3
"""
비상구(출구) 티칭 도구.
로봇을 각 비상구 앞으로 옮긴 뒤 Enter를 눌러 '현재 위치'를 출구로 기록한다.
teach_points.py(점검 지점 티칭)와 동일한 워크플로.
결과는 같은 폴더의 exit_doors.yaml 로 저장되고,
patrol_navigator 가 시작 시 이 파일을 읽어 대피 출구로 사용한다.
(파일이 없으면 patrol_navigator 는 내장 기본 좌표를 사용)

실행:
    python3 teach_doors.py
필요:
    - Unity Play 중 (/odom 이 흐르고 있어야 함)
    - 로봇을 옮기는 방법: Unity WASD(RobotManualController) 또는 RViz 2D Goal Pose
"""
import math
import os
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TeachDoors(Node):
    def __init__(self):
        super().__init__('teach_doors')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got = False
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.got = True


def main():
    rclpy.init()
    node = TeachDoors()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    out_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'exit_doors.yaml')
    doors = []

    print('=' * 50)
    print(' 비상구(출구) 티칭 도구')
    print('=' * 50)
    print('로봇을 비상구 앞으로 옮긴 뒤 Enter → 이름 입력 → 기록.')
    print('※ 문 바로 앞(통과 가능한 지점)에 세우는 게 좋음 — Nav2 목표로 쓰임.')
    print('다 끝나면 이름 자리에 q 입력.\n')

    while True:
        input('[Enter] 현재 위치를 출구로 기록 (q로 종료) ...')
        if not node.got:
            print('  ⏳ 아직 /odom 수신 전. Unity Play 확인 후 다시.\n')
            continue
        print(f'  현재 위치: x={node.x:.2f}, y={node.y:.2f}, yaw={node.yaw:.2f}')
        name = input('  이 출구 이름 (예: exit_door_A, q=끝내기): ').strip()
        if name.lower() == 'q':
            break
        if not name:
            print('  이름이 비어서 건너뜀.\n')
            continue
        doors.append({
            'name': name,
            'x': round(node.x, 3),
            'y': round(node.y, 3),
            'yaw': round(node.yaw, 3),
        })
        print(f'  ✅ "{name}" 기록됨 (총 {len(doors)}개)\n')

    if not doors:
        print('\n기록된 출구가 없어 저장하지 않음. (기존 파일 유지)')
        rclpy.shutdown()
        return

    # YAML 직접 작성 (pyyaml 없이도 동작하도록)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('# 비상구 좌표 (teach_doors.py 티칭으로 생성).\n')
        f.write('# patrol_navigator 가 대피 시 가까운 순서대로 시도.\n')
        f.write('exit_doors:\n')
        for d in doors:
            f.write(f'  - name: "{d["name"]}"\n')
            f.write(f'    x: {d["x"]}\n')
            f.write(f'    y: {d["y"]}\n')
            f.write(f'    yaw: {d["yaw"]}\n')

    print(f'\n💾 저장 완료: {out_path}')
    print(f'   출구 {len(doors)}개 — patrol_navigator 재시작 시 자동 적용')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
