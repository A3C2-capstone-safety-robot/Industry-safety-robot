# Phase 1 완료 보고 — SLAM 기반 맵 생성

**프로젝트**: 산업체 환경 안전을 위한 지능형 자율 점검 및 대피 안내 로봇 시스템  
**완료일**: 2026-05-12  
**담당**: 서강대 2026-1 캡스톤디자인

---

## 완료 기능 목록

### 1. ROS 2 환경 구축 (WSL2)
- WSL2 Ubuntu 24.04 + ROS 2 Jazzy 설치
  - 보고서는 Humble 기준이나 Ubuntu 24.04 환경 → Jazzy 사용
- SLAM Toolbox (`ros-jazzy-slam-toolbox`) 설치
- ros-tcp-endpoint (`main-ros2` 브랜치) 빌드 및 Python 3.12 호환 패치

### 2. Unity ↔ ROS 통신
- ROS-TCP-Connector Unity 패키지 적용
- `ROSConnectionSetup.cs`: 씬 시작 시 WSL2 IP로 자동 연결
  - WSL IP 재부팅 시 변경됨 → `wsl hostname -I`로 확인 후 수정
- ros-tcp-endpoint 실행: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`

### 3. LiDAR 시뮬레이션 (`LidarSensor.cs`)
- Physics.Raycast 360도 수평 스캔, 10Hz
- `sensor_msgs/LaserScan` → `/scan` 토픽 발행
- 좌표 규약: angle=0 → 로컬 +Z(정면), CCW 방향 증가 (ROS 규약 일치)
  - 방향 공식: `(-Mathf.Sin(angle), 0f, Mathf.Cos(angle))`
- 타임스탬프: `DateTimeOffset.UtcNow` 사용 (Unity Time.time 사용 시 TF 룩업 실패)
- 주의: LidarSensor 오브젝트 Y 위치가 장애물 높이 범위 내에 있어야 함
- 장애물 오브젝트 Layer가 `Ignore Raycast`이면 감지 불가 → `Default` 레이어 사용

### 4. 동적 오도메트리 (`OdometryPublisher.cs`)
- 20Hz로 로봇 위치/방향 → `/odom` 및 `/tf` 발행
- Unity → ROS 좌표 변환:
  - `ros.x = unity.z`, `ros.y = -unity.x`
  - `rosYaw = -unity.eulerAngles.y * Deg2Rad`
- `odom → base_link` TF를 동적으로 발행 (정적 발행 대체)
- Player 루트 오브젝트(CharacterController가 있는 곳)에 부착

### 5. TF 트리 구성
```
map
 └── odom
      └── base_link        ← OdometryPublisher.cs (Unity에서 동적 발행)
           └── base_scan   ← WSL static_transform_publisher (고정)
```
- WSL 실행 명령:
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_scan
  ```

### 6. SLAM Toolbox 설정 (`slam_params.yaml`)
- 해상도: 0.05m (5cm) 점유 격자
- `base_frame: base_link`, `scan_topic: /scan`
- 실행:
  ```bash
  ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/min39/slam_params.yaml
  ```
- 주의: `~` 경로 미지원 → 절대 경로 필수

### 7. 자율 이동 (`AutoNavigator.cs`)
- 로봇청소기식 자율 탐색으로 맵 커버리지 확보
- 전방 Physics.Raycast + OnControllerColliderHit 이중 장애물 감지
- Tab 키로 자동/수동 전환
- Player 루트 오브젝트에 부착, CameraMove 자동 비활성화
- 장애물 오브젝트 설정: Collider만 필요 (Rigidbody 불필요)

---

## RViz 맵 색상 의미

| 색상 | 의미 |
|------|------|
| 검정 | 점유 (벽/장애물) |
| 흰색 | 자유 공간 (이동 확인된 영역) |
| 회색 | 미탐색 영역 |

---

## 전체 실행 순서 (재시작 후)

```bash
# WSL 터미널 1
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# WSL 터미널 2
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_scan

# WSL 터미널 3
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/min39/slam_params.yaml

# WSL 터미널 4
ros2 run rviz2 rviz2
# RViz: Fixed Frame → map, Add → /map (Map), /scan (LaserScan)
```
→ Unity Play → Tab 키 → 자동 탐색 시작

---

## 생성된 스크립트 파일

| 파일 | 위치 | 역할 |
|------|------|------|
| `LidarSensor.cs` | `Assets/Scripts/Robot/` | LiDAR 시뮬레이션 및 /scan 발행 |
| `ROSConnectionSetup.cs` | `Assets/Scripts/Robot/` | ROS 연결 초기화 |
| `OdometryPublisher.cs` | `Assets/Scripts/Robot/` | 동적 오도메트리 발행 |
| `AutoNavigator.cs` | `Assets/Scripts/Robot/` | 자율 이동 (Tab 전환) |
| `slam_params.yaml` | 프로젝트 루트 + `/home/min39/` | SLAM 파라미터 |

---

## 다음 단계 — Phase 2 (5/12~5/19)

- [ ] 열화상 퓨전 모듈 (ThermalRenderer): 가상 열원 감지 → `/thermal` 발행
- [ ] 가스 확산 시뮬레이션 + 나방 탐색 알고리즘 (SG-Cast + RMI)
- [ ] Phase 3 (5/20~5/28): 시스템 통합, LLM 보고서, React+FastAPI 대시보드
