# Frontier Exploration Architecture

## Goal

미지 맵에서 SLAM을 진행하면서 frontier 기반으로 탐색한다.
현재 `FrontierExplorer.cs` 하나에 몰린 책임을 다음 단위로 나눈다.

## Recommended Split

1. `ExplorationMapSnapshot`
   `/map` 수신 데이터, 좌표 변환, block/cell 변환, 셀 접근을 담당한다.

2. `FrontierPlanningSettings`
   frontier 검색과 path BFS 설정을 별도 객체로 묶는다.

3. `FrontierRuntimeState`
   blacklist, visited frontier, relax count 같은 런타임 상태를 보관한다.

4. `FrontierPlanner`
   frontier 후보 탐색, 목표 셀 선택, 경로 생성, 실패 후보 blacklist를 담당한다.

5. `FrontierDriver` 또는 `FrontierExplorer`
   Unity MonoBehaviour 레이어.
   ROS 구독, CharacterController 이동, planner 호출, 디버그 로그를 담당한다.

6. `LocalObstacleAvoider`
   waypoint 추종 중 정면/측면 감지와 recovery 행동을 담당한다.

## Runtime Flow

1. `FrontierExplorer`가 `/map`을 받아 `ExplorationMapSnapshot` 갱신
2. snapshot + settings + runtimeState를 `FrontierPlanner`에 전달
3. planner가 `FrontierPlan`을 반환
4. `LocalObstacleAvoider`가 `FrontierPlan.Waypoints`를 따라 이동
5. 성공 시 `VisitedFrontiers` 갱신
6. 실패 시 neighborhood blacklist와 recovery 수행

## Why This Split

- map 품질 문제와 planner 문제를 분리할 수 있다
- frontier 선택 로직만 따로 교체 가능하다
- 나중에 2번 시나리오인 순찰 자율주행에서는 `FrontierPlanner`만 다른 planner로 교체하면 된다
- Unity scene 파라미터와 planner 파라미터를 분리해 테스트하기 쉽다

## Next Refactor Order

1. `FrontierExplorer`의 map 관련 메서드를 `ExplorationMapSnapshot` 사용으로 교체
2. blacklist/visited state를 `FrontierRuntimeState`로 이전
3. `FindFrontiers`, `TryFindGoalCellInBlock`, `BFS`, `PlanPath`를 `FrontierPlanner`로 이동
4. `Navigate`, `DriveToward`, `ProbeCapsule`를 `LocalObstacleAvoider`로 이동
