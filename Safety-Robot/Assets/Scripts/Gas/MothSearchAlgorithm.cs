// MothSearchAlgorithm.cs
// 나방 탐색 알고리즘 (Gradient-Enhanced Surge-Cast-Spiral)
// 농도 그래디언트 + 나방 행동 패턴으로 누출원을 찾아가는 알고리즘
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public enum MothState
{
    Idle,           // 대기 — 가스 미감지
    Surge,          // 추적 — 농도 그래디언트 방향으로 이동
    Cast,           // 탐색 — 좌우 지그재그 (가스 신호 소실 시)
    Spiral,         // 나선 — 고농도 구역에서 누출원 정밀 탐색
    SourceFound     // 누출원 도달
}

public class MothSearchAlgorithm : MonoBehaviour
{
    [Header("센서 참조")]
    public VirtualGasSensor gasSensor;

    // 센서가 자동으로 찾은 DominantPlume 사용
    private GaussianPlumeModel plumeModel
    {
        get { return gasSensor != null ? gasSensor.DominantPlume : null; }
    }

    [Header("이동 모드")]
    public bool useDirectMovement = true;    // true = Unity 직접 이동, false = Nav2 사용
    public float moveSpeed = 2f;             // 직접 이동 속도 (m/s)
    public float rotateSpeed = 120f;         // 회전 속도 (deg/s)

    [Header("Nav2 연동 (useDirectMovement = false일 때)")]
    public string navGoalTopic = "/goal_pose";
    public float goalPublishRate = 1f;

    [Header("탐색 파라미터")]
    public float surgeStepSize = 2f;             // Surge 이동 거리 (m)
    public float castStepSize = 1.5f;            // Cast 좌우 이동 거리 (m)
    public int maxCastSteps = 8;                 // Cast 최대 반복
    public float spiralRadius = 0.5f;            // Spiral 초기 반경 (m)
    public float spiralGrowth = 0.2f;            // Spiral 반경 증가량
    public float sourceThreshold = 80f;          // 누출원 도달 판정 농도 (ppm)

    [Header("감지 임계값")]
    public float detectionThreshold = 3f;        // 가스 감지 최소 농도 (ppm)
    public float highConcThreshold = 40f;        // 고농도 판정 (Spiral 전환)

    [Header("그래디언트 탐색")]
    public float sampleDistance = 1.5f;          // 그래디언트 샘플링 거리 (m)
    public int sampleDirections = 8;             // 샘플링 방향 수

    [Header("디버그")]
    public MothState currentState = MothState.Idle;
    public float debugConcentration = 0f;
    public Vector3 debugGradient = Vector3.zero;

    // 내부 변수
    private ROSConnection ros;
    private float goalTimer = 0f;
    private Vector3 currentGoal;

    // Cast 관련
    private int castCount = 0;
    private int castDirection = 1;
    private Vector3 lastSurgeDirection = Vector3.forward;

    // Spiral 관련
    private float spiralAngle = 0f;
    private float currentSpiralRadius;
    private Vector3 spiralCenter;
    private float bestSpiralConc = 0f;
    private Vector3 bestSpiralPos;

    // 상태 전환 타이머
    private float stateTimer = 0f;
    private float lastDetectionTime = -999f;
    private float signalLostTimeout = 4f;
    private float searchStartTime = -1f;         // 탐색 시작 시각
    private float minSearchTime = 0f;            // 최소 탐색 시간 (초) — 0이면 제한 없음
    private Vector3 searchStartPos;              // 탐색 시작 위치
    private float minSearchDistance = 0f;         // 최소 이동 거리 (m) — 0이면 제한 없음

    // 씬의 모든 플룸 모델 (그래디언트 샘플링용)
    private GaussianPlumeModel[] allPlumes;
    private float plumeRefreshTimer = 0f;

    // CmdVelSubscriber 충돌 방지
    private CmdVelSubscriber cmdVelSub;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(navGoalTopic);
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");

        currentGoal = transform.position;
        currentSpiralRadius = spiralRadius;
        bestSpiralPos = transform.position;
        lastStuckCheckPos = transform.position;

        allPlumes = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);

        // gasSensor 미할당 시 자동 탐색
        if (gasSensor == null)
        {
            gasSensor = GetComponentInChildren<VirtualGasSensor>();
            if (gasSensor == null)
                gasSensor = FindAnyObjectByType<VirtualGasSensor>();
            if (gasSensor != null)
                Debug.Log($"[나방] gasSensor 자동 발견: {gasSensor.gameObject.name}");
            else
                Debug.LogError("[나방] ⚠ gasSensor를 찾을 수 없음! Inspector에서 할당 필요");
        }

        // CmdVelSubscriber 참조 (로봇 이동은 항상 이걸 통해서)
        cmdVelSub = GetComponent<CmdVelSubscriber>();
        if (cmdVelSub == null)
            cmdVelSub = GetComponentInParent<CmdVelSubscriber>();
        if (cmdVelSub == null)
            cmdVelSub = FindAnyObjectByType<CmdVelSubscriber>();
        Debug.Log($"[나방] CmdVelSubscriber: {(cmdVelSub != null ? cmdVelSub.gameObject.name : "없음")}");

        Debug.Log($"[나방] 초기화 완료 — 모드:{(useDirectMovement ? "직접이동" : "Nav2")}, " +
                  $"센서:{(gasSensor != null ? gasSensor.gameObject.name : "NULL")}, " +
                  $"플룸:{allPlumes.Length}개, 감지임계:{detectionThreshold}ppm");
    }

        // 클래스 상단에 추가
    private float stuckTimer = 0f;
    private Vector3 lastStuckCheckPos;
    void Update()
    {
        if (gasSensor == null) return;

        // 주기적으로 플룸 목록 갱신
        plumeRefreshTimer += Time.deltaTime;
        if (plumeRefreshTimer > 5f)
        {
            plumeRefreshTimer = 0f;
            allPlumes = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        }

        debugConcentration = gasSensor.CurrentConcentration;
        stateTimer += Time.deltaTime;
        gradientTimer += Time.deltaTime;  // ← 이 한 줄 추가

        stuckTimer += Time.deltaTime;
        if (stuckTimer > 2f)
        {
            stuckTimer = 0f;
            if (Vector3.Distance(transform.position, lastStuckCheckPos) < 0.5f
                && currentState == MothState.Surge)
            {
                lastSurgeDirection = FindMostOpenDirection();
                Debug.Log($"[나방] 스턱 감지 → 열린 방향 ({lastSurgeDirection.x:F2}, {lastSurgeDirection.z:F2})");
                TransitionTo(MothState.Cast);
            }
            lastStuckCheckPos = transform.position;
        }

        // 3초마다 상태 로그 (디버그)
        plumeRefreshTimer += 0f; // 이미 위에서 증가
        if (Mathf.FloorToInt(Time.time) % 3 == 0 && Mathf.Abs(Time.time - Mathf.Floor(Time.time)) < Time.deltaTime)
        {
            float distToGoal = Vector3.Distance(transform.position, currentGoal);
            Debug.Log($"[나방 상태] {currentState} | 농도:{debugConcentration:F1}ppm | " +
                      $"목표거리:{distToGoal:F1}m | 위치:({transform.position.x:F1},{transform.position.z:F1}) | " +
                      $"목표:({currentGoal.x:F1},{currentGoal.z:F1}) | 모드:{(useDirectMovement ? "직접" : "Nav2")}");
        }

        // 가스 감지 시 마지막 감지 시각 갱신
        if (gasSensor.IsGasDetected(detectionThreshold))
        {
            lastDetectionTime = Time.time;
        }

        // 상태 머신
        switch (currentState)
        {
            case MothState.Idle:    HandleIdle();        break;
            case MothState.Surge:   HandleSurge();       break;
            case MothState.Cast:    HandleCast();        break;
            case MothState.Spiral:  HandleSpiral();      break;
            case MothState.SourceFound: HandleSourceFound(); break;
        }

        // 이동 처리
        if (useDirectMovement)
        {
            // Unity에서 직접 이동 (Nav2 불필요)
            MoveDirectly();
        }
        else
        {
            // Nav2로 목표 좌표 발행
            goalTimer += Time.deltaTime;
            if (goalTimer >= 1f / goalPublishRate)
            {
                goalTimer = 0f;
                PublishNavGoal(currentGoal);
            }
        }
    }

    // /cmd_vel 발행으로 로봇 이동 (CmdVelSubscriber가 실제 이동 담당)
    void MoveDirectly()
    {
        if (currentState == MothState.Idle || currentState == MothState.SourceFound)
        {
            PublishStopCommand();
            return;
        }

        Vector3 toGoal = currentGoal - transform.position;
        toGoal.y = 0f;

        float dist = toGoal.magnitude;
        if (dist < 0.3f)
        {
            PublishStopCommand();
            return;
        }

        // 목표 방향과 로봇 전방의 각도 차이
        float angleToGoal = Vector3.SignedAngle(transform.forward, toGoal, Vector3.up);

        // 선속도: 목표를 거의 바라볼 때만 전진
        float linearX = 0f;
        if (Mathf.Abs(angleToGoal) < 30f)
            linearX = moveSpeed;
        else if (Mathf.Abs(angleToGoal) < 60f)
            linearX = moveSpeed * 0.3f; // 살짝 전진하면서 회전

        // 각속도: 목표 방향으로 회전 (양수=좌회전, 음수=우회전)
        float angularZ = 0f;
        if (Mathf.Abs(angleToGoal) > 5f)
            angularZ = Mathf.Clamp(angleToGoal * Mathf.Deg2Rad * 2f, -2f, 2f);

        // /cmd_vel 발행 → CmdVelSubscriber가 처리
        var twistMsg = new TwistMsg
        {
            linear = new Vector3Msg { x = linearX, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = angularZ }
        };
        ros.Publish("/cmd_vel", twistMsg);
    }

    // ============================================================
    //  STATE HANDLERS
    // ============================================================

    // Idle: 가스 미감지 — 감지하면 Surge
    void HandleIdle()
    {
        if (gasSensor.IsGasDetected(detectionThreshold))
        {
            searchStartTime = Time.time;
            searchStartPos = transform.position;
            TransitionTo(MothState.Surge);
            Debug.Log($"[나방] ★ 탐색 시작! searchStartTime={searchStartTime:F2}, Time.time={Time.time:F2}, 농도:{debugConcentration:F1}ppm");
        }
    }

    // 충분히 탐색했는지 확인 (시간 + 거리 둘 다 충족해야 함)
    bool HasSearchedEnough()
    {
        if (searchStartTime < 0f) return false;

        float elapsed = Time.time - searchStartTime;
        float moved = Vector3.Distance(transform.position, searchStartPos);

        bool timeOk = elapsed >= minSearchTime;
        bool distOk = moved >= minSearchDistance;

        if (!timeOk || !distOk)
        {
            Debug.Log($"[나방] 도달 판정 대기 — 시간:{elapsed:F1}/{minSearchTime}초, 이동:{moved:F1}/{minSearchDistance}m");
        }

        return timeOk && distOk;
    }

    // Surge: 농도 그래디언트를 따라 이동 (핵심!)
    private float gradientTimer = 0f;
    void HandleSurge()
    {
        // 누출원 도달 (Surge에서 최소 3초 그래디언트 추적 후)
        if (gasSensor.CurrentConcentration >= sourceThreshold)
        {
            TransitionTo(MothState.SourceFound);
            Debug.Log("[나방] 누출원 도달!");
            return;
        }

        // 고농도 → Spiral (Surge에서 최소 5초 그래디언트를 따라간 후에만)
        if (gasSensor.CurrentConcentration >= highConcThreshold)
        {
            TransitionTo(MothState.Spiral);
            Debug.Log($"[나방] 고농도({gasSensor.CurrentConcentration:F1}ppm) → Spiral");
            return;
        }

        // 신호 소실 → Cast
        if (Time.time - lastDetectionTime > signalLostTimeout)
        {
            TransitionTo(MothState.Cast);
            Debug.Log("[나방] 신호 소실 → Cast");
            return;
        }

        // ★ 농도 그래디언트 방향으로 이동
        if (gradientTimer >= 1.2f)
        {
            gradientTimer = 0f;

            Vector3 gradDir = ComputeGradientDirection();
            debugGradient = gradDir;

            if (gradDir.magnitude > 0.01f)
            {
                // 그래디언트 방향으로 이동
                currentGoal = ValidateGoal(transform.position + gradDir * surgeStepSize);
                lastSurgeDirection = gradDir;
                Debug.Log($"[나방] Surge → 그래디언트 방향 ({gradDir.x:F2}, {gradDir.z:F2}), 농도:{debugConcentration:F1}");
            }
            else
            {
                // 그래디언트를 못 찾으면 마지막 방향으로 전진
                currentGoal = ValidateGoal(transform.position + lastSurgeDirection * surgeStepSize * 0.5f);
                Debug.Log("[나방] Surge → 그래디언트 불명, 직진");
            }
        }
    }

    // 클래스 상단에 추가
    private float castTimer = 0f;   
    // Cast: 좌우 지그재그 탐색 + 농도 감지 시 그 방향으로 편향
    void HandleCast()
    {
        // Surge 복귀는 stateTimer로 (리셋 안 됨)
        if (stateTimer > 3f && gasSensor.IsGasDetected(detectionThreshold))
        {
            TransitionTo(MothState.Surge);
            Debug.Log("[나방] 신호 재포착 → Surge");
            return;
        }

        // 지그재그는 castTimer로
        castTimer += Time.deltaTime;
        if (castTimer >= 1.5f)
        {
            castTimer = 0f;  // 이것만 리셋

            Vector3 crossDir = Vector3.Cross(lastSurgeDirection, Vector3.up).normalized;
            if (crossDir.magnitude < 0.1f) crossDir = Vector3.right;

            currentGoal = ValidateGoal(transform.position + crossDir * castStepSize * castDirection);
            currentGoal += lastSurgeDirection * 0.5f;

            castCount++;
            castDirection *= -1;

            if (castCount >= maxCastSteps)
            {
                castCount = 0;
                currentGoal = ValidateGoal(transform.position + lastSurgeDirection * surgeStepSize);
                Debug.Log("[나방] Cast 한계 → 전진 후 재탐색");
            }
        }
    }

    // Spiral: 나선 탐색 + 최고 농도 지점 추적
    void HandleSpiral()
    {
        // 누출원 도달 (Spiral에서 2초 탐색 후)
        if (stateTimer > 2f && gasSensor.CurrentConcentration >= sourceThreshold)
        {
            TransitionTo(MothState.SourceFound);
            Debug.Log("[나방] 누출원 도달!");
            return;
        }

        // 농도 완전 소실 → Cast
        if (gasSensor.CurrentConcentration < detectionThreshold && stateTimer > 3f)
        {
            TransitionTo(MothState.Cast);
            Debug.Log("[나방] 농도 소실 → Cast");
            return;
        }

        // 최고 농도 지점 기록
        if (gasSensor.CurrentConcentration > bestSpiralConc)
        {
            bestSpiralConc = gasSensor.CurrentConcentration;
            bestSpiralPos = transform.position;
        }

        // 나선 이동 (spiralCenter 기준)
        spiralAngle += Time.deltaTime * 1.5f;
        currentSpiralRadius += spiralGrowth * Time.deltaTime;

        // 나선 중심을 최고 농도 지점 쪽으로 서서히 이동
        spiralCenter = Vector3.Lerp(spiralCenter, bestSpiralPos, Time.deltaTime * 0.5f);

        float spiralX = spiralCenter.x + Mathf.Cos(spiralAngle) * currentSpiralRadius;
        float spiralZ = spiralCenter.z + Mathf.Sin(spiralAngle) * currentSpiralRadius;

        currentGoal = new Vector3(spiralX, transform.position.y, spiralZ);

        // 나선 한 바퀴 돌았으면 → 최고 농도 지점 쪽으로 이동 시도
        if (spiralAngle > Mathf.PI * 2f && bestSpiralConc > highConcThreshold)
        {
            spiralAngle = 0f;
            currentSpiralRadius = spiralRadius; // 반경 리셋
            spiralCenter = bestSpiralPos;       // 최고 농도 지점 중심으로 재탐색
            bestSpiralConc = 0f;                // 리셋
            Debug.Log($"[나방] Spiral 재조정 → 최고 농도 지점({bestSpiralPos.x:F1}, {bestSpiralPos.z:F1})");
        }
    }

    // SourceFound: 누출원 도달 — 정지 + ROS 알림
    void HandleSourceFound()
    {
        currentGoal = transform.position;

        // 매 프레임 속도 0 발행 (로봇 완전 정지)
        PublishStopCommand();

        // 농도가 sourceThreshold 아래로 떨어지면 → Surge로 복귀 (아직 누출원 아님)
        if (stateTimer > 2f && gasSensor.CurrentConcentration < sourceThreshold)
        {
            TransitionTo(MothState.Surge);
            Debug.Log($"[나방] 농도 부족({gasSensor.CurrentConcentration:F1} < {sourceThreshold}) → Surge 복귀");
            return;
        }

        // 가스 완전 소실 → Idle
        if (stateTimer > 3f && !gasSensor.IsGasDetected(detectionThreshold))
        {
            searchStartTime = -1f;
            TransitionTo(MothState.Idle);
            Debug.Log("[나방] 가스 소실 → Idle 복귀 (재탐색 대기)");
            return;
        }

        if (stateTimer < 0.1f)
        {
            string gasTypeName = plumeModel != null ? plumeModel.gasType.ToString() : "UNKNOWN";

            Debug.Log($"[나방] === 누출원 위치 특정 ===\n" +
                      $"  가스: {gasTypeName}\n" +
                      $"  농도: {gasSensor.CurrentConcentration:F1} ppm\n" +
                      $"  위치: ({transform.position.x:F1}, {transform.position.y:F1}, {transform.position.z:F1})");

            ros.RegisterPublisher<StringMsg>("/moth_search/result");
            ros.Publish("/moth_search/result", new StringMsg
            {
                data = $"SOURCE_FOUND|{gasTypeName}|" +
                       $"{gasSensor.CurrentConcentration:F1}|" +
                       $"{transform.position.x:F2},{transform.position.y:F2},{transform.position.z:F2}"
            });
        }
    }

    // ============================================================
    //  GRADIENT SAMPLING (핵심 — 어디가 더 진한지 탐색)
    // ============================================================

    // 농도 가중평균 방향 — 작은 차이도 누적되어 방향을 잡음
    Vector3 ComputeGradientDirection()
    {
        Vector3 pos = transform.position;
        Vector3 weightedDir = Vector3.zero;

        // 여러 거리에서 샘플링 (가까운 곳 + 먼 곳)
        float[] distances = { sampleDistance, sampleDistance * 2f };

        for (int d = 0; d < distances.Length; d++)
        {
            float dist = distances[d];

            for (int i = 0; i < sampleDirections; i++)
            {
                float angle = (360f / sampleDirections) * i * Mathf.Deg2Rad;
                Vector3 dir = new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
                Vector3 samplePos = pos + dir * dist;

                float conc = SampleConcentrationAt(samplePos);

                // 농도로 방향에 가중치 부여
                // 농도가 높은 방향으로 벡터가 더 길어짐
                weightedDir += dir * conc;
            }
        }

        // 가중 합이 충분히 크면 방향 반환
        if (weightedDir.magnitude > 0.1f)
            return weightedDir.normalized;

        return Vector3.zero;
    }

    // 임의의 좌표에서 모든 활성 누출원의 합산 농도 계산
    float SampleConcentrationAt(Vector3 pos)
    {
        if (allPlumes == null) return 0f;

        float total = 0f;
        for (int i = 0; i < allPlumes.Length; i++)
        {
            if (allPlumes[i] == null || !allPlumes[i].isLeaking) continue;
            total += allPlumes[i].GetConcentration(pos.x, pos.y, pos.z);
        }
        return total;
    }

    // ============================================================
    //  STOP COMMAND
    // ============================================================

    // /cmd_vel에 속도 0 발행 → 로봇 정지
    void PublishStopCommand()
    {
        var stopMsg = new TwistMsg
        {
            linear = new Vector3Msg { x = 0, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = 0 }
        };
        ros.Publish("/cmd_vel", stopMsg);
    }

    // ============================================================
    //  UTILITIES
    // ============================================================

    void TransitionTo(MothState newState)
    {
    // 스턱 타이머 리셋
        stuckTimer = 0f;
        lastStuckCheckPos = transform.position;

        // SourceFound 전환은 반드시 조건 충족 필요 (어디서 호출하든 차단)
        if (newState == MothState.SourceFound)
        {
            float elapsed = searchStartTime > 0f ? Time.time - searchStartTime : 0f;
            float moved = searchStartTime > 0f ? Vector3.Distance(transform.position, searchStartPos) : 0f;

            if (elapsed < minSearchTime)
            {
                Debug.Log($"[나방] SourceFound 차단! 시간:{elapsed:F1}s/{minSearchTime}s");
                return;  // 전환 거부
            }
        }

        Debug.Log($"[나방] 상태 전환: {currentState} → {newState}");
        currentState = newState;
        stateTimer = 0f;

        switch (newState)
        {
            case MothState.Surge:
                // ★ 즉시 첫 목표 설정 (1.2초 대기 없이 바로 움직이도록)
                Vector3 initGrad = ComputeGradientDirection();
                if (initGrad.magnitude > 0.01f)
                {
                    currentGoal = ValidateGoal(transform.position + initGrad * surgeStepSize);
                    lastSurgeDirection = initGrad;
                    Debug.Log($"[나방] Surge 시작 → 즉시 그래디언트({initGrad.x:F2},{initGrad.z:F2})");
                }
                else
                {
                    // 그래디언트 불명이면 가장 가까운 활성 누출원 방향으로
                    Vector3 fallbackDir = FindNearestLeakDirection();
                    if (fallbackDir.magnitude > 0.01f)
                    {
                        currentGoal = ValidateGoal(transform.position + fallbackDir * surgeStepSize);
                        lastSurgeDirection = fallbackDir;
                        Debug.Log($"[나방] Surge 시작 → 누출원 방향({fallbackDir.x:F2},{fallbackDir.z:F2})");
                    }
                    else
                    {
                        currentGoal = ValidateGoal(transform.position + Vector3.forward * surgeStepSize);
                        Debug.Log("[나방] Surge 시작 → 방향 불명, 전진");
                    }
                }
                break;
            case MothState.Cast:
                castCount = 0;
                castDirection = 1;
                break;
            case MothState.Spiral:
                spiralAngle = 0f;
                currentSpiralRadius = spiralRadius;
                spiralCenter = transform.position;
                bestSpiralConc = gasSensor.CurrentConcentration;
                bestSpiralPos = transform.position;
                break;
        }
    }

    // 가장 가까운 활성 누출원 방향 (그래디언트 실패 시 fallback)
    Vector3 FindNearestLeakDirection()
    {
        if (allPlumes == null) return Vector3.zero;

        float minDist = float.MaxValue;
        Vector3 nearestDir = Vector3.zero;

        for (int i = 0; i < allPlumes.Length; i++)
        {
            if (allPlumes[i] == null || !allPlumes[i].isLeaking) continue;
            if (allPlumes[i].leakSource == null) continue;

            Vector3 toSource = allPlumes[i].leakSource.position - transform.position;
            toSource.y = 0f;
            float d = toSource.magnitude;
            if (d < minDist && d > 0.1f)
            {
                minDist = d;
                nearestDir = toSource.normalized;
            }
        }
        return nearestDir;
    }

        // 장애물 검증 — goal이 벽 안이면 벽 앞으로 조정
    Vector3 ValidateGoal(Vector3 goal)
    {
        Vector3 dir = goal - transform.position;
        float dist = dir.magnitude;
        
        if (Physics.SphereCast(transform.position, 0.3f, dir.normalized, out RaycastHit hit, dist))
        {
            if (hit.distance < 1f)
            {
                // 그래디언트 방향과 가장 가까운 열린 방향 탐색
                float bestScore = -2f;
                Vector3 bestGoal = transform.position + transform.forward;
                
                for (int i = 0; i < 12; i++)
                {
                    float angle = (360f / 12) * i;
                    Vector3 testDir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    
                    if (!Physics.SphereCast(transform.position, 0.3f, testDir, out RaycastHit _, surgeStepSize))
                    {
                        float alignment = Vector3.Dot(testDir, dir.normalized);
                        if (alignment > bestScore)
                        {
                            bestScore = alignment;
                            bestGoal = transform.position + testDir * surgeStepSize;
                        }
                    }
                }
                Debug.Log($"[나방] 벽 우회 → 정렬도:{bestScore:F2}");
                return bestGoal;
            }
            return hit.point - dir.normalized * 0.6f;
        }
        return goal;
    }

    // 가장 열린 방향 탐색 (통로 자동 감지)
    Vector3 FindMostOpenDirection()
    {
        float maxDist = 0f;
        Vector3 bestDir = transform.forward;
        
        for (int i = 0; i < 12; i++)
        {
            float angle = (360f / 12) * i;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, 20f))
            {
                if (hit.distance > maxDist)
                {
                    maxDist = hit.distance;
                    bestDir = dir;
                }
            }
            else
            {
                return dir;  // 벽 없음 = 완전 열린 방향
            }
        }
        return bestDir;
    }

    // Nav2 목표 좌표 발행
    void PublishNavGoal(Vector3 position)
{
    int sec = (int)Time.timeAsDouble;
    uint nanosec = (uint)((Time.timeAsDouble - sec) * 1e9);

    // ★ 이동 방향 yaw 계산 (Unity → ROS 좌표계)
    Vector3 dir = position - transform.position;
    float yaw = Mathf.Atan2(-dir.x, dir.z);

    var goalMsg = new PoseStampedMsg
    {
        header = new RosMessageTypes.Std.HeaderMsg
        {
            stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
            {
                sec = sec,
                nanosec = nanosec
            },
            frame_id = "map"
        },
        pose = new PoseMsg
        {
            position = new PointMsg
            {
                x = position.z,
                y = -position.x,
                z = 0
            },
            orientation = new QuaternionMsg
            {
                x = 0,
                y = 0,
                z = Mathf.Sin(yaw / 2f),
                w = Mathf.Cos(yaw / 2f)
            }
        }
    };

    ros.Publish(navGoalTopic, goalMsg);
}

    // Gizmo 디버그
    void OnDrawGizmos()
    {
        switch (currentState)
        {
            case MothState.Surge:
                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position, currentGoal);
                // 그래디언트 방향 표시
                Gizmos.color = Color.white;
                Gizmos.DrawRay(transform.position, debugGradient * 3f);
                break;
            case MothState.Cast:
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position, currentGoal);
                break;
            case MothState.Spiral:
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(spiralCenter, currentSpiralRadius);
                // 최고 농도 지점
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(bestSpiralPos, 0.3f);
                break;
            case MothState.SourceFound:
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(transform.position, 0.5f);
                break;
        }

        // 목표 지점
        Gizmos.color = Color.magenta;
        Gizmos.DrawSphere(currentGoal, 0.3f);
    }
}
