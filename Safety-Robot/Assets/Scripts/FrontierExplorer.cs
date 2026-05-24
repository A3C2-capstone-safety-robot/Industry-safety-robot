using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using System.Collections.Generic;

/// <summary>
/// Tab 키로 ON/OFF하는 Frontier 기반 자율 탐색기
/// /map (OccupancyGrid) 구독 → Phase-1 BFS(Frontier 탐색) → Phase-2 BFS(경로 계획) → 이동
/// 로봇 루트 GameObject에 붙여서 사용 (CharacterController와 동일 오브젝트)
/// </summary>
public class FrontierExplorer : MonoBehaviour
{
    [Header("ROS")]
    public string mapTopic = "/map";

    [Header("Phase 1 — Frontier BFS")]
    public int blockSize = 10;
    public float distanceWeight = 1f;
    public float infoGainWeight = 2f;
    public int safetyRadius = 3;
    public int minInfoGain = 5;

    [Header("Phase 2 — Path BFS")]
    public int pathBlockSize = 3;
    public int pathInflation = 3;
    public int pathWaypoints = 10;

    [Header("Movement")]
    public float moveSpeed = 2f;
    public float rotateSpeed = 120f;
    public float waypointReachDist = 0.5f;
    public float obstacleDetectDist = 1f;
    public float blockDuration = 0.4f;
    public float stuckTimeout = 8f;
    public float escapeDuration = 1.8f;

    // Map
    int mapW, mapH;
    float mapRes;
    Vector2 mapOrig;    // ROS-frame origin
    int[] grid;         // grid[y * mapW + x]: -1=unknown, 0=free, 1-100=occupied

    // Explorer state
    bool isActive;
    List<Vector2Int> blacklist = new List<Vector2Int>();
    int relaxCount;
    const int MAX_RELAXATION = 3;
    float nextSearchTime;

    // Navigation
    List<Vector3> waypoints = new List<Vector3>();
    int wpIdx;
    Vector2Int curTargetBlock;
    float blockedTimer, stuckTimer, escapeTimer;
    bool isEscaping;
    float escapeDir;

    // 이 각도(°) 이내로 정렬됐을 때만 전진 — 더 크면 제자리 회전만 수행
    const float AlignThreshold = 20f;

    CharacterController controller;

    // 수직 속도 (중력 누적), 수평 요청 (매 프레임 DriveToward에서 설정)
    float yVelocity = 0f;
    const float Gravity = -9.81f;
    Vector3 frameHorizontal;   // 이번 프레임의 수평 이동 요청 (Y=0 보장)

    static readonly Vector2Int[] Dirs4 =
    {
        new Vector2Int(1, 0), new Vector2Int(-1, 0),
        new Vector2Int(0, 1), new Vector2Int(0, -1)
    };

    // ==============================================================
    // Unity lifecycle
    // ==============================================================
    void Start()
    {
        controller = GetComponent<CharacterController>();
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>(mapTopic, OnMap);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab)) Toggle();

        // 수평 요청 초기화 — Navigate / DriveToward에서 값을 채움
        frameHorizontal = Vector3.zero;

        if (isActive && grid != null)
        {
            if (waypoints.Count == 0)
                SeekFrontier();
            else
                Navigate();
        }

        // 중력 + 수평을 하나의 Move 호출로 처리 — 항상, 탐색 상태와 무관하게
        CommitMove();
    }

    /// <summary>
    /// 프레임당 단 한 번만 호출되는 실제 이동.
    /// 중력(yVelocity)과 수평(frameHorizontal)을 합쳐 controller.Move 한 번만 호출.
    /// SimpleMove 사용 금지 — 내부 중력 누적이 이 방식과 중복됨.
    /// </summary>
    void CommitMove()
    {
        if (controller == null)
        {
            // CharacterController 없는 경우: 수평 이동만, 중력 없음
            if (frameHorizontal.sqrMagnitude > 0.0001f)
                transform.Translate(frameHorizontal * Time.deltaTime, Space.World);
            return;
        }

        // 착지 판정: 이전 Move에서 바닥에 닿았으면 속도를 -2로 고정 (바닥 밀착)
        if (controller.isGrounded && yVelocity < 0f)
            yVelocity = -2f;
        else
            yVelocity += Gravity * Time.deltaTime;

        // 수평 Y 완전 제거 후 중력 Y 주입 → 하나의 벡터로 Move
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
            grid[i] = msg.data[i]; // sbyte → int; -1=unknown, 0=free, 1-100=occupied
    }

    int Cell(int cx, int cy)
    {
        if (cx < 0 || cy < 0 || cx >= mapW || cy >= mapH) return -1;
        return grid[cy * mapW + cx];
    }

    // ==============================================================
    // Coordinate conversions
    // ==============================================================
    // Unity → ROS: rosX = unity.z,  rosY = -unity.x
    // ROS   → Unity: unity.z = rosX, unity.x = -rosY

    Vector2Int WorldToCell(Vector3 p) => new Vector2Int(
        Mathf.FloorToInt((p.z - mapOrig.x) / mapRes),
        Mathf.FloorToInt((-p.x - mapOrig.y) / mapRes));

    Vector3 CellToWorld(int cx, int cy)
    {
        float rosX = (cx + 0.5f) * mapRes + mapOrig.x;
        float rosY = (cy + 0.5f) * mapRes + mapOrig.y;
        return new Vector3(-rosY, transform.position.y, rosX);
    }

    Vector2Int CellToBlock(Vector2Int c, int bs) => new Vector2Int(c.x / bs, c.y / bs);

    Vector2Int BlockToCenter(Vector2Int b, int bs) =>
        new Vector2Int(b.x * bs + bs / 2, b.y * bs + bs / 2);

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
    // Phase 1 — Frontier BFS
    // ==============================================================
    bool HasUnknownNeighborBlock(Vector2Int block, int bs)
    {
        foreach (var d in Dirs4)
        {
            var nc = BlockToCenter(block + d, bs);
            // out-of-bounds → Cell returns -1 (unknown) → 맵 경계도 Frontier 후보로 처리
            if (Cell(nc.x, nc.y) < 0) return true;
        }
        return false;
    }

    struct FrontierInfo { public Vector2Int block; public float score; }

    List<FrontierInfo> FindFrontiers()
    {
        int bs = blockSize;
        int bW = mapW / bs, bH = mapH / bs;
        var startBlock = CellToBlock(WorldToCell(transform.position), bs);

        var visited = new HashSet<Vector2Int>();
        var q = new Queue<Vector2Int>();
        var result = new List<FrontierInfo>();

        if (startBlock.x < 0 || startBlock.y < 0 ||
            startBlock.x >= bW || startBlock.y >= bH) return result;

        visited.Add(startBlock);
        q.Enqueue(startBlock);

        while (q.Count > 0)
        {
            var cur = q.Dequeue();
            var cc = BlockToCenter(cur, bs);
            int v = Cell(cc.x, cc.y);

            // Frontier 조건: free 블록 + unknown 이웃 블록 존재 + 안전 + 블랙리스트 미포함
            if (v == 0 && HasUnknownNeighborBlock(cur, bs)
                && !blacklist.Contains(cur) && IsSafe(cc))
            {
                int info = CountUnknown(cc, bs);
                if (info >= minInfoGain)
                {
                    float dist = Vector3.Distance(transform.position, CellToWorld(cc.x, cc.y));
                    result.Add(new FrontierInfo
                    {
                        block = cur,
                        score = -distanceWeight * dist + infoGainWeight * info
                    });
                }
            }

            // 인접 블록 확장: 중앙 셀 값 ≤ 50이면 통과 (free 또는 unknown)
            foreach (var d in Dirs4)
            {
                var nb = cur + d;
                if (visited.Contains(nb)) continue;
                if (nb.x < 0 || nb.y < 0 || nb.x >= bW || nb.y >= bH) continue;
                var nc = BlockToCenter(nb, bs);
                if (Cell(nc.x, nc.y) <= 50) { visited.Add(nb); q.Enqueue(nb); }
            }
        }

        return result;
    }

    // ==============================================================
    // Phase 2 — Path BFS
    // ==============================================================
    List<Vector2Int> BFS(Vector2Int start, Vector2Int goal, int bs, bool inflated)
    {
        int bW = mapW / bs, bH = mapH / bs;
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

                // 1차: free(=0) + 팽창 여유, 2차: 점유만 아니면(≤50) 통과
                bool ok = inflated ? (v == 0 && IsClear(nc.x, nc.y)) : (v <= 50);
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

        // 1차: 팽창 적용 → 실패 시 팽창 없이 재시도 (좁은 통로 대비)
        var blocks = BFS(sb, gb, bs, true) ?? BFS(sb, gb, bs, false);
        if (blocks == null) return null;

        var pts = new List<Vector3>();
        foreach (var b in blocks)
        {
            var bc = BlockToCenter(b, bs);
            pts.Add(CellToWorld(bc.x, bc.y));
        }

        // 균등 간격으로 최대 pathWaypoints 개 추출
        if (pts.Count <= pathWaypoints) return pts;
        var sampled = new List<Vector3>();
        float step = (float)(pts.Count - 1) / (pathWaypoints - 1);
        for (int i = 0; i < pathWaypoints; i++)
            sampled.Add(pts[Mathf.RoundToInt(i * step)]);
        return sampled;
    }

    // ==============================================================
    // Exploration orchestration
    // ==============================================================
    void Toggle()
    {
        isActive = !isActive;
        waypoints.Clear();
        isEscaping = false;
        blockedTimer = stuckTimer = escapeTimer = 0f;
        Debug.Log($"[FrontierExplorer] {(isActive ? "ON" : "OFF")}");
    }

    void SeekFrontier()
    {
        if (Time.time < nextSearchTime) return;
        nextSearchTime = Time.time + 0.5f; // 탐색 속도 제한 (0.5초마다 1회)

        var candidates = FindFrontiers();

        if (candidates.Count == 0)
        {
            relaxCount++;
            blacklist.Clear();
            Debug.Log($"[FrontierExplorer] 후보 없음 — 블랙리스트 초기화 ({relaxCount}/{MAX_RELAXATION})");
            candidates = FindFrontiers();

            if (candidates.Count == 0 || relaxCount >= MAX_RELAXATION)
            {
                Debug.Log("[FrontierExplorer] 탐색 완료 — 모든 Frontier 소진");
                isActive = false;
                return;
            }
        }

        // 최고 점수 Frontier 선택
        candidates.Sort((a, b) => b.score.CompareTo(a.score));
        var best = candidates[0];
        curTargetBlock = best.block;
        var gc = BlockToCenter(curTargetBlock, blockSize);

        var newPath = PlanPath(CellToWorld(gc.x, gc.y));
        if (newPath == null || newPath.Count == 0)
        {
            Debug.Log($"[FrontierExplorer] 경로 없음: block {curTargetBlock} → 블랙리스트 추가");
            blacklist.Add(curTargetBlock);
            return;
        }

        waypoints = newPath;
        wpIdx = 0;
        stuckTimer = blockedTimer = 0f;
        Debug.Log($"[FrontierExplorer] 목표 block {curTargetBlock}, 점수 {best.score:F1}, 웨이포인트 {waypoints.Count}개");
    }

    // ==============================================================
    // Movement
    // ==============================================================

    void Navigate()
    {
        if (wpIdx >= waypoints.Count) { waypoints.Clear(); return; }

        Vector3 wp = new Vector3(waypoints[wpIdx].x, transform.position.y, waypoints[wpIdx].z);

        if (Vector3.Distance(transform.position, wp) < waypointReachDist)
        {
            wpIdx++;
            blockedTimer = stuckTimer = 0f;
            return;
        }

        stuckTimer += Time.deltaTime;
        if (stuckTimer > stuckTimeout)
        {
            Debug.Log($"[FrontierExplorer] Stuck 타임아웃 — block {curTargetBlock} 블랙리스트");
            blacklist.Add(curTargetBlock);
            waypoints.Clear();
            stuckTimer = blockedTimer = 0f;
            isEscaping = false;
            return;
        }

        if (isEscaping)
        {
            escapeTimer += Time.deltaTime;
            // Space.World: 로봇이 기울어져도 항상 월드 Y축 기준 제자리 회전
            transform.Rotate(Vector3.up * escapeDir * rotateSpeed * Time.deltaTime, Space.World);
            // frameHorizontal은 zero 유지 — CommitMove가 중력만 처리
            if (escapeTimer >= escapeDuration)
            {
                isEscaping = false;
                escapeTimer = blockedTimer = 0f;
            }
            return;
        }

        // 장애물 감지: forward의 수평 성분만 사용 (Y 제거)
        Vector3 flatFwd = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
        bool obstacleAhead = flatFwd.sqrMagnitude > 0.001f &&
                             Physics.Raycast(transform.position + Vector3.up * 0.3f,
                                             flatFwd, obstacleDetectDist);

        if (obstacleAhead)
        {
            blockedTimer += Time.deltaTime;
            if (blockedTimer >= blockDuration)
            {
                isEscaping = true;
                escapeTimer = 0f;
                escapeDir = Random.value > 0.5f ? 1f : -1f;
            }
            // frameHorizontal zero 유지 — CommitMove가 중력만 처리
        }
        else
        {
            blockedTimer = 0f;
            DriveToward(wp);
        }
    }

    void DriveToward(Vector3 target)
    {
        Vector3 dir = Vector3.ProjectOnPlane(target - transform.position, Vector3.up).normalized;
        if (dir.sqrMagnitude < 0.001f) return;

        // 항상 제자리 회전
        transform.rotation = Quaternion.RotateTowards(
            transform.rotation,
            Quaternion.LookRotation(dir),
            rotateSpeed * Time.deltaTime);

        // transform.forward에서 Y 명시적 제거
        Vector3 flatForward = new Vector3(transform.forward.x, 0f, transform.forward.z).normalized;
        if (flatForward.sqrMagnitude < 0.001f) return;

        // 목표 방향으로 AlignThreshold° 이내일 때만 전진 요청
        float angle = Vector3.Angle(flatForward, dir);
        if (angle < AlignThreshold)
            frameHorizontal = flatForward * moveSpeed;
        // else: frameHorizontal 유지 (zero) — CommitMove가 중력만 처리
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
