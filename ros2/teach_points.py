#!/usr/bin/env python3
"""
점검 지점 티칭(teach) 도구.
로봇을 각 기계 앞으로 옮긴 뒤 Enter를 눌러 '현재 위치'를 점검 지점으로 기록한다.
실제 산업용 점검 로봇의 teach-and-repeat 방식과 동일한 워크플로.
결과는 같은 폴더의 inspection_points.yaml 로 저장된다.

실행:
    python3 teach_points.py
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


class TeachPoints(Node):
    def __init__(self):
        super().__init__('teach_points')
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
    node = TeachPoints()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    out_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'inspection_points.yaml')
    points = []

    print('=' * 50)
    print(' 점검 지점 티칭 도구')
    print('=' * 50)
    print('로봇을 기계 앞으로 옮긴 뒤 Enter → 이름 입력 → 기록.')
    print('다 끝나면 이름 자리에 q 입력.\n')

    while True:
        input('[Enter] 현재 위치 기록 (q로 종료) ...')
        if not node.got:
            print('  ⏳ 아직 /odom 수신 전. Unity Play 확인 후 다시.\n')
            continue
        print(f'  현재 위치: x={node.x:.2f}, y={node.y:.2f}, yaw={node.yaw:.2f}')
        name = input('  이 지점 이름 (q=끝내기): ').strip()
        if name.lower() == 'q':
            break
        if not name:
            print('  이름이 비어서 건너뜀.\n')
            continue
        points.append({
            'name': name,
            'x': round(node.x, 3),
            'y': round(node.y, 3),
            'yaw': round(node.yaw, 3),
        })
        print(f'  ✅ "{name}" 기록됨 (총 {len(points)}개)\n')

    # YAML 직접 작성 (pyyaml 없이도 동작하도록)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('# 점검 지점 (티칭으로 생성). patrol_navigator 가 이 순서대로 방문.\n')
        f.write('inspection_points:\n')
        for p in points:
            f.write(f'  - name: "{p["name"]}"\n')
            f.write(f'    x: {p["x"]}\n')
            f.write(f'    y: {p["y"]}\n')
            f.write(f'    yaw: {p["yaw"]}\n')

    print(f'\n💾 저장 완료: {out_path}')
    print(f'   점검 지점 {len(points)}개')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
