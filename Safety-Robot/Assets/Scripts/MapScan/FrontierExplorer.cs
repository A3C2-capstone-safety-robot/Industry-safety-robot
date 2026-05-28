using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using System.Collections.Generic;

/// <summary>
/// Tab 키로 ON/OFF하는 Frontier 기반 자율 탐색기
/// /map (OccupancyGrid) 구독 → Phase-1 BFS(Frontier 탐색) → Phase-2 BFS(경로 계획) → 이동
/// 로봇 루트 GameObject에 붙여서 사용 (CharacterController와 동일 오브젝트)
/// </summary>
[RequireComponent(typeof(CharacterController))]
public class FrontierExplorer : MonoBehaviour
{
    [Header("ROS")]
    public string mapTopic = "/map";
    [Tooltip("Tab으로 다시 켰을 때 이전 방문 frontier 기록을 유지할지 여부")]
    public bool keepVisitedFrontiersOnRestart = false;

    [Header("Phase 1 — Frontier BFS")]
    public int blockSize = 10;
    public float distanceWeight = 1f;
    public float infoGainWeight = 2f;
    public int safetyRadius = 3;
    public int minInfoGain = 5;
    [Tooltip("방문 완료된 frontier 근처 블록의 스코어 감점 가중치")]
    public float visitedPenaltyWeight = 3f;
    [Tooltip("방문 감점을 적용할 블록 거리 (맨해튼)")]
    public int visitedPenaltyRadius = 2;

    [Header("Phase 2 — Path BFS")]
    public int pathBlockSize = 3;
    public int pathInflation = 3;
    public int pathWaypoints = 10;
    [Tooltip("목표 셀 주변 회전 여유를 검사할 반경")]
    public int goalClearanceRadius = 1;
    [Tooltip("목표 셀 주변 free 이웃 최소 개수")]
    public int minGoalOpenNeighbors = 5;
    [Tooltip("더 탁 트인 목표 셀을 선호하는 가중치")]
    public float goalOpennessWeight = 1.5f;

    [Header("Movement")]
    public float moveSpeed = 2f;
    public float rotateSpeed = 120f;
    public float waypointReachDist = 0.5f;
    [Tooltip("이 각도(°) 이내로 정렬되면 전진")]
    public float alignThreshold = 30f;
    [Tooltip("장애물 감지 거리 — moveSpeed의 1.5배 이상 권장")]
    public float obstacleDetectDist = 3f;
    public float blockDuration = 0.4f;
    public float stuckTimeout = 8f;
    public float escapeDuration = 1.8f;

    [Header("Obstacle Avoidance")]
    [Tooltip("전방 좌/우 보조 감지 각도")]
    public float sideProbeAngle = 45f;
    [Tooltip("보조 감지 거리 (정면보다 짧게)")]
    public float sideProbeDistance = 2f;
    [Tooltip("측면 장애물 감지 시 반대쪽 조향 각도")]
    public float avoidSteerAngle = 35f;
    [Tooltip("정면 장애물 감지 초기에 전진을 얼마나 유지할지 (0~1)")]
    [Range(0.1f, 1f)]
    public float obstacleCreepSpeedFactor = 0.45f;
    [Tooltip("회피 종료 직후 즉시 재회전에 빠지지 않도록 하는 유예 시간")]
    public float postEscapeRecovery = 0.35f;

    [Header("Body Clearance — CapsuleCast")]
    [Tooltip("경로 물리 검증(CapsuleCast) 사용 여부")]
    public bool usePhysicalValidation = true;
    [Tooltip("CapsuleCast에서 무시할 레이어")]
    public LayerMask castIgnoreLayers;
    [Tooltip("CharacterController 크기에서 CapsuleCast 몸체를 자동 계산")]
    public bool autoFitBodyFromController = true;
    [Tooltip("로봇 몸체 반지름 (CapsuleCast용)")]
    public float bodyRadius = 0.3f;
    [Tooltip("캡슐 하단 중심 높이 (지면 기준)")]
    public float bodyBottom = 0.1f;
    [Tooltip("캡슐 상단 중심 높이 (지면 기준)")]
    public float bodyTop = 1.0f;

    // Map
    int mapW, mapH;
    float mapRes;
    Vector2 mapOrig;
    int[] grid;
    ExplorationMapSnapshot mapSnapshot;

    // Explorer state
    bool isActive;
    readonly FrontierRuntimeState runtimeState = new FrontierRuntimeState();
    readonly FrontierPlanner planner = new FrontierPlanner();
    const int MAX_RELAXATION = 3;
    const int MaxPlanAttemptsPerSearch = 8;
    const int BlacklistNeighborRadius = 1;
    float nextSearchTime;
    bool warnedNoMap;
    float nextNavigateDebugTime;

    // Navigation
    List<Vector3> waypoints = new List<Vector3>();
    int wpIdx;
    Vector2Int curTargetBlock;
    float blockedTimer, stuckTimer, escapeTimer;
    bool isEscaping;
    float escapeDir;

    const float FloorClearanceMargin = 0.05f;

    CharacterController controller;

    float yVelocity = 0f;
    const float Gravity = -9.81f;
    Vector3 frameHorizontal;
    float postEscapeTimer;

    static readonly Vector2Int[] Dirs4 =
    {
        new Vector2Int(1, 0), new Vector2Int(-1, 0),
        new Vector2Int(0, 1), new Vector2Int(0, -1)
    };

    List<Vector2Int> blacklist => runtimeState.Blacklist;
    HashSet<Vector2Int> visitedFrontiers => runtimeState.VisitedFrontiers;
    int relaxCount
    {
        get => runtimeState.RelaxCount;
        set => runtimeState.RelaxCount = value;
    }

    // ==============================================================
    // Unity lifecycle
    // ==============================================================
    void Start()
    {
        controller = GetComponent<CharacterController>();
        FitBodyClearanceFromController();
        ValidateLayerSetup();
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>(mapTopic, OnMap);
    }

    FrontierPlanningSettings CreatePlanningSettings()
    {
        var settings = new FrontierPlanningSettings
        {
            blockSize = blockSize,
            distanceWeight = distanceWeight,
            infoGainWeight = infoGainWeight,
            safetyRadius = safetyRadius,
            minInfoGain = minInfoGain,
            visitedPenaltyWeight = visitedPenaltyWeight,
            visitedPenaltyRadius = visitedPenaltyRadius,
            pathBlockSize = pathBlockSize,
            pathInflation = pathInflation,
            pathWaypoints = pathWaypoints,
            maxPlanAttemptsPerSearch = MaxPlanAttemptsPerSearch,
            blacklistNeighborRadius = BlacklistNeighborRadius,
            goalClearanceRadius = goalClearanceRadius,
            minGoalOpenNeighbors = minGoalOpenNeighbors,
            goalOpennessWeight = goalOpennessWeight
        };

        settings.Clamp();
        return settings;
    }

    // [Codex Fix #1] 로봇이 Default(0) 레이어에 있으면 경고
    void ValidateLayerSetup()
    {
        if (gameObject.layer == 0)
        {
            Debug.LogWarning(
                "[FrontierExplorer] 로봇이 Default(0) 레이어에 있습니다. "
                + "벽도 Default라면 CapsuleCast가 자식 콜라이더에 오탐할 수 있습니다. "
                + "로봇 전용 레이어(예: Robot)를 만들고 castIgnoreLayers에 추가하세요.", this);
        }
    }

    // [Codex Fix #2] autoFit이 꺼져 있어도 CC보다 작으면 강제 보정
    void FitBodyClearanceFromController()
    {
        if (controller == null)
        {
            Debug.LogWarning("[FrontierExplorer] CharacterController 없음 — body clearance 자동 보정 건너뜀", this);
            return;
        }

        if (autoFitBodyFromController)
        {
            bodyRadius = controller.radius;
            float halfHeight = controller.height * 0.5f;
            float centerY = controller.center.y;
            bodyBottom = centerY - halfHeight + bodyRadius;
            bodyTop = centerY + halfHeight - bodyRadius;

            if (bodyTop < bodyBottom)
            {
                bodyBottom = centerY;
                bodyTop = centerY;
            }

            ApplyBodyClearanceClamp();

            Debug.Log(
                "[FrontierExplorer] Auto-fit body clearance "
                + "(radius=" + bodyRadius.ToString("F2")
                + ", bottom=" + bodyBottom.ToString("F2")
                + ", top=" + bodyTop.ToString("F2") + ")", this);
        }
        else
        {
            // autoFit OFF일 때: 수동 값이 CC보다 작으면 경고 후 강제 보정
            if (bodyRadius < controller.radius)
            {
                Debug.LogWarning(
                    "[FrontierExplorer] bodyRadius(" + bodyRadius.ToString("F2")
                    + ") < CC.radius(" + controller.radius.ToString("F2")
                    + ") — 자동 보정합니다.", this);
                bodyRadius = controller.radius;
            }

            float ccBottom = controller.center.y - controller.height * 0.5f + controller.radius;
            float ccTop = controller.center.y + controller.height * 0.5f - controller.radius;
            if (ccTop < ccBottom) ccTop = ccBottom;

            if (bodyBottom > ccBottom || bodyTop < ccTop)
            {
                Debug.LogWarning(
                    "[FrontierExplorer] CapsuleCast 높이가 CC보다 작습니다 — 자동 보정합니다.", this);
                bodyBottom = ccBottom;
                bodyTop = ccTop;
            }

            ApplyBodyClearanceClamp();
        }
    }

    void ApplyBodyClearanceClamp()
    {
        float minBottom = bodyRadius + FloorClearanceMargin;
        if (bodyBottom < minBottom)
            bodyBottom = minBottom;
        if (bodyTop < bodyBottom)
            bodyTop = bodyBottom;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab)) Toggle();

        frameHorizontal = Vector3.zero;

        if (isActive && mapSnapshot != null)
        {
            if (waypoints.Count == 0)
                SeekFrontier();
            else
                Navigate();
        }

        CommitMove();
    }

    void CommitMove()
    {
        if (controller == null)
        {
            if (frameHorizontal.sqrMagnitude > 0.0001f)
                transform.Translate(frameHorizontal * Time.deltaTime, Space.World);
            return;
        }

        if (controller.isGrounded && yVelocity < 0f)
            yVelocity = -2f;
        else
            yVelocity += Gravity * Time.deltaTime;

        Vector3 motion = new Vector3(frameHorizontal.x, 0f, frameHorizontal.z) * Time.deltaTime;
        motion.y = yVelocity * Time.deltaTime;
        controller.Move(motion);
    }

    // ==============================================================
    // Map reception
    // ==============================================================
    void OnMap(OccupancyGridMsg msg)
    {
        mapW = (int)msg.info.width;
        mapH = (int)msg.info.height;
        mapRes = msg.info.resolution;
        mapOrig = new Vector2((float)msg.info.origin.position.x,
                              (float)msg.info.origin.position.y);

        grid = new int[msg.data.Length];
        for (int i = 0; i < msg.data.Length; i++)
            grid[i] = msg.data[i];

        mapSnapshot = ExplorationMapSnapshot.FromOccupancyGrid(msg);

        warnedNoMap = false;
    }

    int Cell(int cx, int cy)
    {
        if (cx < 0 || cy < 0 || cx >= mapW || cy >= mapH) return -1;
        return grid[cy * mapW + cx];
    }

    // ==============================================================
    // Coordinate conversions
    // ==============================================================
    Vector2Int WorldToCell(Vector3 p)
    {
        return new Vector2Int(
            Mathf.FloorToInt((p.z - mapOrig.x) / mapRes),
            Mathf.FloorToInt((-p.x - mapOrig.y) / mapRes));
    }

    Vector3 CellToWorld(int cx, int cy)
    {
        float rosX = (cx + 0.5f) * mapRes + mapOrig.x;
        float rosY = (cy + 0.5f) * mapRes + mapOrig.y;
        return new Vector3(-rosY, transform.position.y, rosX);
    }

    // [Codex Fix #5] 음수 좌표에서도 올바르게 동작하는 floor-division
    static int FloorDiv(int a, int b)
    {
        return (a >= 0) ? a / b : (a - b + 1) / b;
    }

    Vector2Int CellToBlock(Vector2Int c, int bs)
    {
        return new Vector2Int(FloorDiv(c.x, bs), FloorDiv(c.y, bs));
    }

    // [Codex Fix #4] 가장자리 블록의 중심이 맵 범위를 벗어나지 않도록 clamp
    Vector2Int BlockToCenter(Vector2Int b, int bs)
    {
        int cx = Mathf.Clamp(b.x * bs + bs / 2, 0, mapW > 0 ? mapW - 1 : 0);
        int cy = Mathf.Clamp(b.y * bs + bs / 2, 0, mapH > 0 ? mapH - 1 : 0);
        return new Vector2Int(cx, cy);
    }

    // ==============================================================
    // Safety / clearance helpers
    // ==============================================================
    bool IsSafe(Vector2Int c)
    {
        for (int dy = -safetyRadius; dy <= safetyRadius; dy++)
        for (int dx = -safetyRadius; dx <= safetyRadius; dx++)
        {
            if (dx * dx + dy * dy > safetyRadius * safetyRadius) continue;
            if (Cell(c.x + dx, c.y + dy) > 50) return false;
        }
        return true;
    }

    bool IsClear(int cx, int cy)
    {
        for (int dy = -pathInflation; dy <= pathInflation; dy++)
        for (int dx = -pathInflation; dx <= pathInflation; dx++)
            if (Cell(cx + dx, cy + dy) > 50) return false;
        return true;
    }

    int CountUnknown(Vector2Int c, int radius)
    {
        int n = 0;
        for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            if (Cell(c.x + dx, c.y + dy) < 0) n++;
        return n;
    }

    // ==============================================================
    // Physical clearance check (CapsuleCast)
    // ==============================================================

    bool IsPhysicallyClear(Vector3 from, Vector3 to)
    {
        Vector3 dir = to - from;
        dir.y = 0f;
        float dist = dir.magnitude;
        if (dist < 0.01f) return true;
        dir /= dist;

        ApplyBodyClearanceClamp();
        Vector3 p1 = from + Vector3.up * bodyBottom;
        Vector3 p2 = from + Vector3.up * bodyTop;
        LayerMask mask = GetCastMask();

        return !Physics.CapsuleCast(p1, p2, bodyRadius, dir, dist,
                                    mask, QueryTriggerInteraction.Ignore);
    }

    // [Codex Fix #1] gameObject.layer를 ignore에 OR하지 않음.
    // 로봇이 Default(0)이면 벽(Default)까지 무시되는 치명적 버그 수정.
    // CapsuleCast는 시작점 내부의 콜라이더를 자동으로 무시하므로 별도 처리 불필요.
    LayerMask GetCastMask()
    {
        return ~castIgnoreLayers;
    }

    // 다방향 장애물 감지 헬퍼
    bool ProbeCapsule(Vector3 direction, float distance)
    {
        if (direction.sqrMagnitude < 0.001f) return false;

        ApplyBodyClearanceClamp();
        Vector3 p1 = transform.position + Vector3.up * bodyBottom;
        Vector3 p2 = transform.position + Vector3.up * bodyTop;
        LayerMask mask = GetCastMask();

        return Physics.CapsuleCast(p1, p2, bodyRadius,
                                   direction.normalized, distance,
                                   mask, QueryTriggerInteraction.Ignore);
    }

    // ==============================================================
    // Phase 1 — Frontier BFS
    // ==============================================================
    bool HasUnknownNeighborBlock(Vector2Int block, int bs)
    {
        foreach (var d in Dirs4)
        {
            var nc = BlockToCenter(block + d, bs);
            if (Cell(nc.x, nc.y) < 0) return true;
        }
        return false;
    }

    struct FrontierInfo
    {
        public Vector2Int block;
        public float score;
    }

    int MinDistToVisited(Vector2Int block)
    {
        int best = int.MaxValue;
        foreach (var v in visitedFrontiers)
        {
            int d = Mathf.Abs(block.x - v.x) + Mathf.Abs(block.y - v.y);
            if (d < best) best = d;
        }
        return best;
    }

    List<FrontierInfo> FindFrontiers()
    {
        int bs = blockSize;
        // [Codex Fix #4] ceiling 나눗셈으로 remainder strip 포함
        int bW = (mapW + bs - 1) / bs;
        int bH = (mapH + bs - 1) / bs;
        var startBlock = CellToBlock(WorldToCell(transform.position), bs);

        var visited = new HashSet<Vector2Int>();
        var q = new Queue<Vector2Int>();
        var result = new List<FrontierInfo>();

        if (startBlock.x < 0 || startBlock.y < 0 ||
            startBlock.x >= bW || startBlock.y >= bH)
        {
            Debug.LogWarning("[FrontierExplorer] 시작 block이 맵 범위 밖");
            return result;
        }

        visited.Add(startBlock);
        q.Enqueue(startBlock);

        while (q.Count > 0)
        {
            var cur = q.Dequeue();
            var cc = BlockToCenter(cur, bs);
            int v = Cell(cc.x, cc.y);

            if (v == 0 && HasUnknownNeighborBlock(cur, bs)
                && !blacklist.Contains(cur)
                && !visitedFrontiers.Contains(cur)
                && IsSafe(cc))
            {
                int info = CountUnknown(cc, bs);
                if (info >= minInfoGain)
                {
                    float dist = Vector3.Distance(transform.position, CellToWorld(cc.x, cc.y));
                    float score = -distanceWeight * dist + infoGainWeight * info;

                    int dv = MinDistToVisited(cur);
                    if (dv <= visitedPenaltyRadius)
                        score -= visitedPenaltyWeight * (visitedPenaltyRadius - dv + 1);

                    result.Add(new FrontierInfo { block = cur, score = score });
                }
            }

            foreach (var d in Dirs4)
            {
                var nb = cur + d;
                if (visited.Contains(nb)) continue;
                if (nb.x < 0 || nb.y < 0 || nb.x >= bW || nb.y >= bH) continue;
                var nc = BlockToCenter(nb, bs);
                if (Cell(nc.x, nc.y) == 0)
                {
                    visited.Add(nb);
                    q.Enqueue(nb);
                }
            }
        }

        return result;
    }

    bool TryFindGoalCellInBlock(Vector2Int block, int bs, out Vector2Int goalCell)
    {
        int minX = Mathf.Max(0, block.x * bs);
        int minY = Mathf.Max(0, block.y * bs);
        int maxX = Mathf.Min(mapW - 1, (block.x + 1) * bs - 1);
        int maxY = Mathf.Min(mapH - 1, (block.y + 1) * bs - 1);

        Vector2Int robotCell = WorldToCell(transform.position);
        bool found = false;
        int bestUnknown = -1;
        float bestDistSq = float.MaxValue;
        goalCell = Vector2Int.zero;

        for (int y = minY; y <= maxY; y++)
        for (int x = minX; x <= maxX; x++)
        {
            if (Cell(x, y) != 0) continue;

            var c = new Vector2Int(x, y);
            if (!IsSafe(c)) continue;

            int unknown = CountUnknown(c, 1);
            float distSq = (c - robotCell).sqrMagnitude;

            if (!found || unknown > bestUnknown || (unknown == bestUnknown && distSq < bestDistSq))
            {
                found = true;
                bestUnknown = unknown;
                bestDistSq = distSq;
                goalCell = c;
            }
        }

        return found;
    }

    void BlacklistFrontierNeighborhood(Vector2Int center, int radius)
    {
        for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
        {
            var b = new Vector2Int(center.x + dx, center.y + dy);
            if (!blacklist.Contains(b))
                blacklist.Add(b);
        }
    }

    // ==============================================================
    // Phase 2 — Path BFS
    // ==============================================================
    List<Vector2Int> BFS(Vector2Int start, Vector2Int goal, int bs, bool inflated)
    {
        // [Codex Fix #4] ceiling 나눗셈
        int bW = (mapW + bs - 1) / bs;
        int bH = (mapH + bs - 1) / bs;
        var visited = new HashSet<Vector2Int>();
        var q = new Queue<Vector2Int>();
        var parent = new Dictionary<Vector2Int, Vector2Int>();

        visited.Add(start);
        q.Enqueue(start);

        while (q.Count > 0)
        {
            var cur = q.Dequeue();
            if (cur == goal)
            {
                var path = new List<Vector2Int>();
                for (var n = cur; n != start; n = parent[n]) path.Add(n);
                path.Add(start);
                path.Reverse();
                return path;
            }

            foreach (var d in Dirs4)
            {
                var nb = cur + d;
                if (visited.Contains(nb)) continue;
                if (nb.x < 0 || nb.y < 0 || nb.x >= bW || nb.y >= bH) continue;

                var nc = BlockToCenter(nb, bs);
                int v = Cell(nc.x, nc.y);

                bool ok = inflated ? (v == 0 && IsClear(nc.x, nc.y)) : (v == 0);
                if (!ok) continue;

                visited.Add(nb);
                parent[nb] = cur;
                q.Enqueue(nb);
            }
        }

        return null;
    }

    List<Vector3> PlanPath(Vector3 goalWorld)
    {
        int bs = pathBlockSize;
        var sb = CellToBlock(WorldToCell(transform.position), bs);
        var gb = CellToBlock(WorldToCell(goalWorld), bs);

        var blocks = BFS(sb, gb, bs, true) ?? BFS(sb, gb, bs, false);
        if (blocks == null) return null;

        var pts = new List<Vector3>();
        foreach (var b in blocks)
        {
            var bc = BlockToCenter(b, bs);
            pts.Add(CellToWorld(bc.x, bc.y));
        }

        if (usePhysicalValidation)
        {
            for (int i = 0; i < pts.Count - 1; i++)
            {
                if (!IsPhysicallyClear(pts[i], pts[i + 1]))
                {
                    Debug.Log("[FrontierExplorer] 경로 물리검증 실패: 구간 " + i + " 통과 불가");
                    return null;
                }
            }
        }

        if (pts.Count > pathWaypoints)
        {
            var sampled = new List<Vector3>();
            float step = (float)(pts.Count - 1) / (pathWaypoints - 1);
            for (int i = 0; i < pathWaypoints; i++)
                sampled.Add(pts[Mathf.RoundToInt(i * step)]);
            pts = sampled;
        }

        return pts;
    }

    // ==============================================================
    // Exploration orchestration
    // ==============================================================

    // [Codex Fix #3] Toggle 시 blacklist/relaxCount도 초기화
    void Toggle()
    {
        isActive = !isActive;
        waypoints.Clear();
        isEscaping = false;
        postEscapeTimer = 0f;
        nextSearchTime = 0f;
        blockedTimer = stuckTimer = escapeTimer = 0f;

        if (isActive)
        {
            runtimeState.ResetForNewRun(keepVisitedFrontiersOnRestart);
        }

        Debug.Log("[FrontierExplorer] " + (isActive ? "ON" : "OFF"));

        if (isActive && mapSnapshot == null)
        {
            warnedNoMap = true;
            Debug.LogWarning("[FrontierExplorer] /map 데이터를 아직 받지 못함");
        }
    }

    void SeekFrontier()
    {
        if (mapSnapshot == null)
        {
            if (!warnedNoMap)
            {
                warnedNoMap = true;
                Debug.LogWarning("[FrontierExplorer] /map 데이터 없음 — 대기 중");
            }
            return;
        }

        if (Time.time < nextSearchTime) return;
        nextSearchTime = Time.time + 0.5f;

        FrontierPlanningSettings settings = CreatePlanningSettings();
        var candidates = planner.FindCandidates(
            mapSnapshot,
            transform.position,
            transform.position.y,
            settings,
            runtimeState);
        Debug.Log("[FrontierExplorer] 탐색 tick: candidates=" + candidates.Count
            + ", blacklist=" + blacklist.Count
            + ", visited=" + visitedFrontiers.Count);

        if (candidates.Count == 0)
        {
            relaxCount++;
            blacklist.Clear();
            Debug.Log("[FrontierExplorer] 후보 없음 — 블랙리스트 초기화 (" + relaxCount + "/" + MAX_RELAXATION + ")");
            candidates = planner.FindCandidates(
                mapSnapshot,
                transform.position,
                transform.position.y,
                settings,
                runtimeState);

            if (candidates.Count == 0 || relaxCount >= MAX_RELAXATION)
            {
                Debug.Log("[FrontierExplorer] 탐색 완료 — 모든 Frontier 소진");
                isActive = false;
                return;
            }
        }

        candidates.Sort((a, b) => b.Score.CompareTo(a.Score));

        int attempts = 0;
        foreach (var candidate in candidates)
        {
            if (attempts >= settings.maxPlanAttemptsPerSearch) break;
            if (blacklist.Contains(candidate.Block)) continue;
            attempts++;

            System.Func<Vector3, Vector3, bool> segmentValidator =
                usePhysicalValidation ? new System.Func<Vector3, Vector3, bool>(IsPhysicallyClear) : null;

            FrontierPlan plan = planner.TryBuildPlan(
                mapSnapshot,
                transform.position,
                transform.position.y,
                settings,
                runtimeState,
                candidate,
                segmentValidator);

            if (plan == null)
            {
                Debug.Log("[FrontierExplorer] 경로 없음: block " + candidate.Block + " 주변 블랙리스트");
                continue;
            }

            curTargetBlock = plan.TargetBlock;
            waypoints = plan.Waypoints;
            wpIdx = 0;
            stuckTimer = blockedTimer = 0f;
            relaxCount = 0; // [Codex Fix #3] 성공 시 relaxCount 리셋
            Debug.Log("[FrontierExplorer] 목표 block " + curTargetBlock
                + ", 점수 " + plan.Score.ToString("F1")
                + ", wp " + waypoints.Count + "개");
            return;
        }

        if (attempts > 0)
        {
            Debug.Log("[FrontierExplorer] 이번 탐색 tick에서 유효 경로를 찾지 못함");
        }
    }

    // ==============================================================
    // Movement
    // ==============================================================

    void Navigate()
    {
        if (wpIdx >= waypoints.Count)
        {
            visitedFrontiers.Add(curTargetBlock);
            Debug.Log("[FrontierExplorer] Frontier " + curTargetBlock
                + " 방문 완료 (총 " + visitedFrontiers.Count + "개)");
            waypoints.Clear();
            return;
        }

        Vector3 wp = new Vector3(waypoints[wpIdx].x, transform.position.y, waypoints[wpIdx].z);
        float distToWp = Vector3.Distance(transform.position, wp);

        if (distToWp < waypointReachDist)
        {
            wpIdx++;
            blockedTimer = stuckTimer = 0f;
            return;
        }

        stuckTimer += Time.deltaTime;
        if (stuckTimer > stuckTimeout)
        {
            Debug.Log("[FrontierExplorer] Stuck 타임아웃 — " + curTargetBlock + " 블랙리스트");
            blacklist.Add(curTargetBlock);
            waypoints.Clear();
            stuckTimer = blockedTimer = 0f;
            isEscaping = false;
            postEscapeTimer = 0f;
            return;
        }

        if (postEscapeTimer > 0f)
            postEscapeTimer = Mathf.Max(0f, postEscapeTimer - Time.deltaTime);

        if (isEscaping)
        {
            escapeTimer += Time.deltaTime;
            transform.Rotate(Vector3.up * escapeDir * rotateSpeed * Time.deltaTime, Space.World);
            if (escapeTimer >= escapeDuration)
            {
                isEscaping = false;
                postEscapeTimer = postEscapeRecovery;
                escapeTimer = blockedTimer = 0f;
            }
            return;
        }

        // 3방향 CapsuleCast: 정면 + 좌/우
        Vector3 flatFwd = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
        bool obstacleCenter = false;
        bool obstacleLeft = false;
        bool obstacleRight = false;

        if (flatFwd.sqrMagnitude > 0.001f)
        {
            obstacleCenter = ProbeCapsule(flatFwd, obstacleDetectDist);

            Vector3 leftDir = Quaternion.Euler(0, -sideProbeAngle, 0) * flatFwd;
            Vector3 rightDir = Quaternion.Euler(0, sideProbeAngle, 0) * flatFwd;
            obstacleLeft = ProbeCapsule(leftDir, sideProbeDistance);
            obstacleRight = ProbeCapsule(rightDir, sideProbeDistance);
        }

        if (obstacleCenter)
        {
            blockedTimer += Time.deltaTime;

            // 정면에 잠깐 걸린 경우에는 즉시 제자리 회전 대신 살짝 조향하며 전진 유지
            float steerAngle = avoidSteerAngle * 0.5f;
            Vector3 steerDir = flatFwd;
            if (obstacleLeft && !obstacleRight)
                steerDir = Quaternion.Euler(0, steerAngle, 0) * flatFwd;
            else if (obstacleRight && !obstacleLeft)
                steerDir = Quaternion.Euler(0, -steerAngle, 0) * flatFwd;

            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                Quaternion.LookRotation(steerDir),
                rotateSpeed * Time.deltaTime);

            Vector3 newFwd = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
            if (newFwd.sqrMagnitude > 0.001f)
                frameHorizontal = newFwd * moveSpeed * obstacleCreepSpeedFactor;

            // 충분히 오래 막혔을 때만 제자리 회피로 승격
            if (postEscapeTimer <= 0f && blockedTimer >= blockDuration)
            {
                Vector3 toWp = Vector3.ProjectOnPlane(wp - transform.position, Vector3.up);
                float cross = Vector3.Cross(flatFwd, toWp).y;

                if (obstacleLeft && !obstacleRight)
                    escapeDir = 1f;
                else if (obstacleRight && !obstacleLeft)
                    escapeDir = -1f;
                else
                    escapeDir = cross >= 0f ? 1f : -1f;

                isEscaping = true;
                escapeTimer = 0f;
                frameHorizontal = Vector3.zero;
            }
        }
        else if (obstacleLeft || obstacleRight)
        {
            // 측면만 장애물: 반대쪽 조향 + 전진
            Vector3 steerDir;
            if (obstacleLeft && !obstacleRight)
                steerDir = Quaternion.Euler(0, avoidSteerAngle, 0) * flatFwd;
            else if (obstacleRight && !obstacleLeft)
                steerDir = Quaternion.Euler(0, -avoidSteerAngle, 0) * flatFwd;
            else
            {
                // 양쪽 다 막힘 — 좁은 통로, 직진
                DriveToward(wp);
                return;
            }

            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                Quaternion.LookRotation(steerDir),
                rotateSpeed * Time.deltaTime);

            Vector3 newFwd = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
            if (newFwd.sqrMagnitude > 0.001f)
                frameHorizontal = newFwd * moveSpeed;

            blockedTimer = 0f;
        }
        else
        {
            blockedTimer = 0f;
            DriveToward(wp);
        }

        if (Time.time >= nextNavigateDebugTime)
        {
            Debug.Log("[FrontierExplorer] nav: wp=" + (wpIdx + 1) + "/" + waypoints.Count
                + ", dist=" + distToWp.ToString("F2")
                + ", obstacle(C/L/R)=" + obstacleCenter + "/" + obstacleLeft + "/" + obstacleRight
                + ", escaping=" + isEscaping
                + ", stuck=" + stuckTimer.ToString("F2"));
            nextNavigateDebugTime = Time.time + 1f;
        }
    }

    void DriveToward(Vector3 target)
    {
        Vector3 dir = Vector3.ProjectOnPlane(target - transform.position, Vector3.up).normalized;
        if (dir.sqrMagnitude < 0.001f) return;

        transform.rotation = Quaternion.RotateTowards(
            transform.rotation,
            Quaternion.LookRotation(dir),
            rotateSpeed * Time.deltaTime);

        Vector3 flatForward = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
        if (flatForward.sqrMagnitude < 0.001f) return;

        float angle = Vector3.Angle(flatForward, dir);
        if (angle < alignThreshold)
            frameHorizontal = flatForward * moveSpeed;
    }

    // ==============================================================
    // Editor visualization
    // ==============================================================
    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Count == 0) return;
        Gizmos.color = Color.cyan;
        for (int i = 0; i < waypoints.Count - 1; i++)
            Gizmos.DrawLine(waypoints[i], waypoints[i + 1]);
        if (wpIdx < waypoints.Count)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(waypoints[wpIdx], 0.3f);
        }
    }
}
