using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Frontier-based exploration — /map 구독 → BFS로 도달 가능한 Frontier만 탐색
/// Player 루트(CharacterController)에 부착. Tab 키로 ON/OFF
/// </summary>
[RequireComponent(typeof(CharacterController))]
public class FrontierExplorer : MonoBehaviour
{
    [Header("이동 설정")]
    public float moveSpeed       = 1.5f;
    public float rotateSpeed     = 120f;
    public float arrivalDistance = 1.5f;

    [Header("레이캐스트 설정")]
    public LayerMask obstacleLayerMask = ~0;  // Inspector에서 Robot 레이어 체크 해제

    [Header("Frontier 탐색 설정")]
    public int   safetyRadius   = 3;
    public float stuckTimeout   = 8f;
    [Range(0f, 5f)] public float distanceWeight = 1.0f;
    [Range(0f, 5f)] public float infoGainWeight = 2.0f;
    public int   infoGainRadius = 15;
    public int   blockSize      = 10;   // frontier BFS 블록 크기 (셀)

    [Header("경로 계획 설정")]
    public int   pathBlockSize  = 3;    // 경로 BFS 블록 크기 (frontier보다 세밀하게)
    public int   pathInflation  = 3;    // 경로상 장애물 팽창 반경 (셀) — 벽과의 최소 거리
    public int   pathWaypoints  = 10;   // 최대 중간 웨이포인트 수

    CharacterController controller;
    MonoBehaviour cameraMove;

    bool    isActive    = false;
    bool    hasTarget   = false;
    bool    isSearching = false;
    Vector3 currentTarget;
    Vector2Int currentTargetCell;
    float   stuckTimer  = 0f;

    bool  isEscaping   = false;
    float escapeTimer  = 0f;
    float escapeRotDir = 1f;
    float blockedTime  = 0f;
    const float BLOCK_TO_ESCAPE = 0.4f;
    const float ESCAPE_DURATION = 1.8f;
    const int   FAIL_BL_RADIUS  = 2;

    int    mapWidth, mapHeight;
    float  mapResolution;
    float  originX, originY;
    sbyte[] mapData;
    bool   mapUpdated = false;

    List<Vector2Int> failedFrontiers   = new List<Vector2Int>();
    int              relaxationAttempts = 0;
    const int        MAX_RELAXATION     = 3;

    List<Vector3> currentPath = new List<Vector3>();

    // ── 좌표 변환 ────────────────────────────────────────────────

    void WorldToCell(Vector3 worldPos, out int col, out int row)
    {
        float rosX =  worldPos.z;
        float rosY = -worldPos.x;
        col = Mathf.Clamp(Mathf.RoundToInt((rosX - originX) / mapResolution), 0, mapWidth  - 1);
        row = Mathf.Clamp(Mathf.RoundToInt((rosY - originY) / mapResolution), 0, mapHeight - 1);
    }

    Vector3 CellToWorld(int col, int row)
    {
        float rosX = originX + (col + 0.5f) * mapResolution;
        float rosY = originY + (row + 0.5f) * mapResolution;
        return new Vector3(-rosY, transform.position.y, rosX);
    }

    // ── 초기화 ──────────────────────────────────────────────────

    void Start()
    {
        controller = GetComponent<CharacterController>();
        foreach (var mb in GetComponentsInChildren<MonoBehaviour>())
            if (mb.GetType().Name == "PlayerCamera") cameraMove = mb;

        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>("/map", OnMapReceived);
    }

    void OnMapReceived(OccupancyGridMsg msg)
    {
        mapWidth      = (int)msg.info.width;
        mapHeight     = (int)msg.info.height;
        mapResolution = msg.info.resolution;
        originX       = (float)msg.info.origin.position.x;
        originY       = (float)msg.info.origin.position.y;
        mapData       = msg.data;
        mapUpdated    = true;
    }

    // ── 메인 루프 ────────────────────────────────────────────────

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            isActive = !isActive;
            if (cameraMove != null) cameraMove.enabled = !isActive;
            hasTarget          = false;
            isSearching        = false;
            isEscaping         = false;
            blockedTime        = 0f;
            relaxationAttempts = 0;
            failedFrontiers.Clear();
            currentPath.Clear();
            if (isActive && mapData == null)
                Debug.LogWarning("[Frontier] /map 미수신 — ROS SLAM이 실행 중인지 확인하세요");
            Debug.Log(isActive ? "[Frontier] ON — Tab키로 종료" : "[Frontier] OFF");
        }

        if (!isActive || mapData == null) return;

        mapUpdated = false;
        if (!hasTarget && !isSearching)
            StartCoroutine(FindFrontier());

        if (!hasTarget) return;

        Vector3 navTarget = currentPath.Count > 0 ? currentPath[0] : currentTarget;

        float dist = Vector2.Distance(
            new Vector2(transform.position.x, transform.position.z),
            new Vector2(navTarget.x, navTarget.z));

        if (dist < arrivalDistance)
        {
            if (currentPath.Count > 0) { currentPath.RemoveAt(0); stuckTimer = 0f; return; }
            hasTarget = false;
            stuckTimer = 0f;
            currentPath.Clear();
            return;
        }

        stuckTimer += Time.deltaTime;
        if (stuckTimer > stuckTimeout)
        {
            Debug.Log("[Frontier] Stuck → 다음 목표 탐색");
            failedFrontiers.Add(currentTargetCell);
            hasTarget  = false;
            stuckTimer = 0f;
            currentPath.Clear();
            return;
        }

        MoveToward(navTarget);
    }

    // ── Phase 1: Frontier 탐색 BFS (블록 단위, free+unknown 통과) ─

    IEnumerator FindFrontier()
    {
        isSearching = true;

        int BLOCK = Mathf.Max(1, blockSize);
        int bW = Mathf.Max(1, mapWidth  / BLOCK);
        int bH = Mathf.Max(1, mapHeight / BLOCK);

        WorldToCell(transform.position, out int rCC, out int rCR);
        int rbC = Mathf.Clamp(rCC / BLOCK, 0, bW - 1);
        int rbR = Mathf.Clamp(rCR / BLOCK, 0, bH - 1);

        var visited    = new bool[bW * bH];
        var queue      = new Queue<Vector2Int>();
        var candidates = new List<(Vector2Int bCell, float dist)>();

        int[] dC = {  0,  0, -1, 1 };
        int[] dR = { -1,  1,  0, 0 };

        visited[rbR * bW + rbC] = true;
        queue.Enqueue(new Vector2Int(rbC, rbR));
        int iter = 0;

        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();

            for (int d = 0; d < 4; d++)
            {
                int nc = cur.x + dC[d];
                int nr = cur.y + dR[d];
                if (nc < 0 || nc >= bW || nr < 0 || nr >= bH) continue;
                if (visited[nr * bW + nc]) continue;
                visited[nr * bW + nc] = true;

                int cC = nc * BLOCK + BLOCK / 2;
                int cR = nr * BLOCK + BLOCK / 2;
                if (cC >= mapWidth || cR >= mapHeight) continue;
                sbyte cv = mapData[cR * mapWidth + cC];

                if (cv > 50) continue;  // 점유 블록 → 통과 불가

                queue.Enqueue(new Vector2Int(nc, nr));

                if (cv == 0)  // free 블록 → frontier 후보 판정
                {
                    bool isFrontier = false;
                    for (int d2 = 0; d2 < 4 && !isFrontier; d2++)
                    {
                        int fc = nc + dC[d2], fr = nr + dR[d2];
                        if (fc < 0 || fc >= bW || fr < 0 || fr >= bH) continue;
                        int fC = fc * BLOCK + BLOCK / 2;
                        int fR = fr * BLOCK + BLOCK / 2;
                        if (fC >= mapWidth || fR >= mapHeight) continue;
                        if (mapData[fR * mapWidth + fC] < 0) isFrontier = true;
                    }
                    if (isFrontier)
                        candidates.Add((new Vector2Int(nc, nr),
                            Vector2.Distance(new Vector2(nc, nr), new Vector2(rbC, rbR))));
                }
            }

            if (++iter % 200 == 0) yield return null;
        }

        // ── 후보 스코어링 ────────────────────────────────────────
        float      bestScore = float.MinValue;
        Vector2Int best      = Vector2Int.zero;
        bool       found     = false;

        foreach (var (bCell, dist) in candidates)
        {
            int cC = bCell.x * BLOCK + BLOCK / 2;
            int cR = bCell.y * BLOCK + BLOCK / 2;
            if (cC >= mapWidth || cR >= mapHeight) continue;
            if (dist * BLOCK * mapResolution < arrivalDistance) continue;
            if (!IsCellSafe(cC, cR)) continue;

            bool blacklisted = false;
            foreach (var f in failedFrontiers)
                if (Mathf.Abs(f.x - bCell.x) + Mathf.Abs(f.y - bCell.y) < FAIL_BL_RADIUS)
                { blacklisted = true; break; }
            if (blacklisted) continue;

            float infoGain = CountUnknown(cC, cR, infoGainRadius);
            if (infoGain < 5) continue;
            float distMeters = dist * BLOCK * mapResolution;
            float score      = -distanceWeight * distMeters + infoGainWeight * infoGain;
            if (score > bestScore) { bestScore = score; best = bCell; found = true; }
        }

        if (found)
        {
            int tC = best.x * BLOCK + BLOCK / 2;
            int tR = best.y * BLOCK + BLOCK / 2;
            currentTarget     = CellToWorld(tC, tR);
            currentTargetCell = best;

            // ── Phase 2: 맵 기반 경로 계획 (free 셀만 통과) ────────
            yield return StartCoroutine(PlanPath(rCC, rCR, tC, tR));

            hasTarget          = true;
            stuckTimer         = 0f;
            relaxationAttempts = 0;
            Debug.Log($"[Frontier] 목표 → ({currentTarget.x:F1}, {currentTarget.z:F1}), " +
                      $"score={bestScore:F1}, 후보={candidates.Count}, 경로={currentPath.Count}개 웨이포인트");
        }
        else
        {
            if (failedFrontiers.Count > 0)
            {
                relaxationAttempts++;
                if (relaxationAttempts >= MAX_RELAXATION)
                {
                    failedFrontiers.Clear();
                    relaxationAttempts = 0;
                    Debug.Log("[Frontier] 블랙리스트 전체 초기화 → 재탐색");
                }
                else
                {
                    failedFrontiers.RemoveAt(0);
                    Debug.Log($"[Frontier] 블랙리스트 완화 ({relaxationAttempts}/{MAX_RELAXATION}) → 재탐색");
                }
            }
            else
            {
                hasTarget = false;
                isActive  = false;
                if (cameraMove != null) cameraMove.enabled = true;
                Debug.Log("[Frontier] 탐색 완료 — 모든 Frontier 소진");
            }
        }

        isSearching = false;
    }

    // ── Phase 2: 맵 기반 경로 계획 BFS (free 셀만 통과 + 장애물 팽창) ─

    IEnumerator PlanPath(int startC, int startR, int goalC, int goalR)
    {
        currentPath.Clear();

        int PB = Mathf.Max(1, pathBlockSize);
        int pbW = Mathf.Max(1, mapWidth  / PB);
        int pbH = Mathf.Max(1, mapHeight / PB);

        int pbSC = Mathf.Clamp(startC / PB, 0, pbW - 1);
        int pbSR = Mathf.Clamp(startR / PB, 0, pbH - 1);
        int pbGC = Mathf.Clamp(goalC  / PB, 0, pbW - 1);
        int pbGR = Mathf.Clamp(goalR  / PB, 0, pbH - 1);

        int startIdx = pbSR * pbW + pbSC;
        int goalIdx  = pbGR * pbW + pbGC;

        var pathVisited = new bool[pbW * pbH];
        var pathParent  = new int[pbW * pbH];
        for (int i = 0; i < pathParent.Length; i++) pathParent[i] = -1;

        var pathQueue = new Queue<int>();
        pathVisited[startIdx] = true;
        pathQueue.Enqueue(startIdx);

        int[] dC = {  0,  0, -1, 1 };
        int[] dR = { -1,  1,  0, 0 };
        int iter = 0;
        bool pathFound = false;

        while (pathQueue.Count > 0)
        {
            int cur = pathQueue.Dequeue();
            if (cur == goalIdx) { pathFound = true; break; }

            int curC = cur % pbW;
            int curR = cur / pbW;

            for (int d = 0; d < 4; d++)
            {
                int nc = curC + dC[d];
                int nr = curR + dR[d];
                if (nc < 0 || nc >= pbW || nr < 0 || nr >= pbH) continue;
                int nIdx = nr * pbW + nc;
                if (pathVisited[nIdx]) continue;

                int cc = nc * PB + PB / 2;
                int cr = nr * PB + PB / 2;
                if (cc >= mapWidth || cr >= mapHeight) continue;

                // free 셀만 통과 + 장애물에서 pathInflation 이상 떨어져야 함
                sbyte cv = mapData[cr * mapWidth + cc];
                if (cv != 0) continue;                // 반드시 free
                if (!IsCellClearOfObstacles(cc, cr, pathInflation)) continue;

                pathVisited[nIdx] = true;
                pathParent[nIdx]  = cur;
                pathQueue.Enqueue(nIdx);
            }

            if (++iter % 500 == 0) yield return null;
        }

        if (!pathFound)
        {
            // 팽창 없이 재시도 (좁은 통로 대비)
            for (int i = 0; i < pathVisited.Length; i++) { pathVisited[i] = false; pathParent[i] = -1; }
            pathVisited[startIdx] = true;
            pathQueue.Clear();
            pathQueue.Enqueue(startIdx);
            iter = 0;

            while (pathQueue.Count > 0)
            {
                int cur = pathQueue.Dequeue();
                if (cur == goalIdx) { pathFound = true; break; }

                int curC = cur % pbW;
                int curR = cur / pbW;

                for (int d = 0; d < 4; d++)
                {
                    int nc = curC + dC[d];
                    int nr = curR + dR[d];
                    if (nc < 0 || nc >= pbW || nr < 0 || nr >= pbH) continue;
                    int nIdx = nr * pbW + nc;
                    if (pathVisited[nIdx]) continue;

                    int cc = nc * PB + PB / 2;
                    int cr = nr * PB + PB / 2;
                    if (cc >= mapWidth || cr >= mapHeight) continue;
                    if (mapData[cr * mapWidth + cc] > 50) continue;  // 점유만 제외

                    pathVisited[nIdx] = true;
                    pathParent[nIdx]  = cur;
                    pathQueue.Enqueue(nIdx);
                }

                if (++iter % 500 == 0) yield return null;
            }
        }

        if (!pathFound) { Debug.Log("[Frontier] 경로 없음 — 직선 이동 시도"); yield break; }

        // 경로 역추적
        var rawPath = new List<int>();
        int idx = goalIdx;
        while (idx != startIdx && idx != -1)
        {
            rawPath.Add(idx);
            idx = pathParent[idx];
        }
        rawPath.Reverse();

        // 균등 간격 웨이포인트 추출
        int step = Mathf.Max(1, rawPath.Count / pathWaypoints);
        for (int i = step; i < rawPath.Count - 1; i += step)
        {
            int pIdx = rawPath[i];
            int pc   = pIdx % pbW;
            int pr   = pIdx / pbW;
            currentPath.Add(CellToWorld(pc * PB + PB / 2, pr * PB + PB / 2));
        }
    }

    // ── 헬퍼 함수 ────────────────────────────────────────────────

    bool IsCellSafe(int col, int row)
    {
        for (int dr = -safetyRadius; dr <= safetyRadius; dr++)
        for (int dc = -safetyRadius; dc <= safetyRadius; dc++)
        {
            int r = row + dr, c = col + dc;
            if (r < 0 || r >= mapHeight || c < 0 || c >= mapWidth) continue;
            if (mapData[r * mapWidth + c] > 50) return false;
        }
        return true;
    }

    bool IsCellClearOfObstacles(int col, int row, int radius)
    {
        for (int dr = -radius; dr <= radius; dr++)
        for (int dc = -radius; dc <= radius; dc++)
        {
            int r = row + dr, c = col + dc;
            if (r < 0 || r >= mapHeight || c < 0 || c >= mapWidth) continue;
            if (mapData[r * mapWidth + c] > 50) return false;
        }
        return true;
    }

    float CountUnknown(int col, int row, int radius)
    {
        int count = 0;
        for (int dr = -radius; dr <= radius; dr++)
        for (int dc = -radius; dc <= radius; dc++)
        {
            int r = row + dr, c = col + dc;
            if (r < 0 || r >= mapHeight || c < 0 || c >= mapWidth) continue;
            if (mapData[r * mapWidth + c] < 0) count++;
        }
        return count;
    }

    // ── 이동 제어 ────────────────────────────────────────────────

    void MoveToward(Vector3 target)
    {
        Vector3 sensorOrigin = transform.position + Vector3.up * 0.5f;

        if (isEscaping)
        {
            transform.Rotate(0f, escapeRotDir * 150f * Time.deltaTime, 0f);
            if (!Physics.Raycast(sensorOrigin, transform.forward, 1.0f, obstacleLayerMask))
                controller.SimpleMove(transform.forward * moveSpeed);
            escapeTimer += Time.deltaTime;
            if (escapeTimer >= ESCAPE_DURATION)
            { isEscaping = false; blockedTime = 0f; }
            return;
        }

        Vector3 toTarget = new Vector3(
            target.x - transform.position.x, 0f,
            target.z - transform.position.z).normalized;

        transform.rotation = Quaternion.RotateTowards(
            transform.rotation,
            Quaternion.LookRotation(toTarget),
            rotateSpeed * Time.deltaTime);

        if (Vector3.Angle(transform.forward, toTarget) > 35f)
        { blockedTime = 0f; return; }

        if (Physics.Raycast(sensorOrigin, transform.forward, 0.9f, obstacleLayerMask))
        {
            blockedTime += Time.deltaTime;
            if (blockedTime >= BLOCK_TO_ESCAPE)
            {
                isEscaping   = true;
                escapeTimer  = 0f;
                escapeRotDir = Random.value > 0.5f ? 1f : -1f;
            }
        }
        else
        {
            controller.SimpleMove(transform.forward * moveSpeed);
            blockedTime = 0f;
        }
    }

    void OnControllerColliderHit(ControllerColliderHit hit)
    {
        if (!isActive || !hasTarget) return;
        if (hit.normal.y > 0.5f) return;
        stuckTimer += 2f;
    }

    void OnDrawGizmosSelected()
    {
        if (!isActive) return;
        if (hasTarget)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(currentTarget, 0.4f);
        }
        Gizmos.color = Color.yellow;
        Vector3 prev = transform.position;
        foreach (var wp in currentPath)
        {
            Gizmos.DrawSphere(wp, 0.25f);
            Gizmos.DrawLine(prev, wp);
            prev = wp;
        }
        if (hasTarget)
            Gizmos.DrawLine(prev, currentTarget);
    }
}
