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
    ReturnToBest,   // 수렴 후 최고 농도 지점으로 복귀
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
    public float sourceThreshold = 80f;          // 누출원 도달 판정 농도 (ppm) — 절대 임계값(보조)
    public float localMaxEpsilon = 0.5f;         // 국소최대 판정 여유(주변보다 이만큼 안 높아도 정점 인정)
    public float convergeSeconds = 6f;           // 이 시간 동안 농도 개선 없으면 최고점으로 복귀
    public float arrivalRadius = 1f;             // bestPos 도달 판정 반경 (m) — 이동단위(2m)·σ 고려
    public int maxReturnRetries = 2;             // bestPos 도착 후 재탐색(Surge) 허용 횟수

    // 수렴 판정용 — 탐색 중 최고 농도 지점 기억
    private float bestConc = 0f;
    private Vector3 bestPos;
    private float lastImproveTime = 0f;
    private float emaConc = 0f;                  // 노이즈 억제용 지수이동평균 농도
    private int returnCount = 0;                 // ReturnToBest → Surge 재시도 횟수

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
        // 누출원 발견 결과 토픽 — 미리 등록해야 발견 순간 발행이 안 날아감
        ros.RegisterPublisher<StringMsg>("/moth_search/result");

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

        // 노이즈 억제 — 지수이동평균(EMA) 농도 (순간 노이즈 ±2ppm이 bestConc를 부풀리는 것 방지)
        emaConc = Mathf.Lerp(emaConc, gasSensor.CurrentConcentration, Time.deltaTime * 1.5f);

        // 탐색 중 최고 농도 지점 추적 (의미 있는 개선만 갱신 → 수렴 판정)
        if (currentState == MothState.Surge || currentState == MothState.Cast
            || currentState == MothState.Spiral || currentState == MothState.ReturnToBest)
        {
            if (emaConc > bestConc + Mathf.Max(0.3f, bestConc * 0.05f))
            {
                bestConc = emaConc;
                bestPos = transform.position;
                lastImproveTime = Time.time;
            }
        }

        // Cast(신호 소실 탐색) 중에는 수렴 시계 정지 — 헤매는 시간이 조기 수렴으로 이어지지 않게
        if (currentState == MothState.Cast)
            lastImproveTime += Time.deltaTime;

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
            case MothState.ReturnToBest: HandleReturnToBest(); break;
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
            bestConc = 0f;                       // 최고점 추적 초기화
            bestPos = transform.position;
            lastImproveTime = Time.time;
            emaConc = gasSensor.CurrentConcentration;
            returnCount = 0;
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
        // 현재 위치가 농도 정점 → 누출원 확정
        if (IsAtLocalMaximum())
        {
            TransitionTo(MothState.SourceFound);
            Debug.Log("[나방] 누출원 도달! (국소 최대)");
            return;
        }

        // 수렴(개선 없음) → 그 자리에서 멈추지 말고 최고 농도 지점으로 복귀
        if (HasConverged())
        {
            TransitionTo(MothState.ReturnToBest);
            Debug.Log($"[나방] 수렴 → 최고점({bestPos.x:F1},{bestPos.z:F1}) 복귀");
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
        // 현재 위치가 농도 정점 → 누출원 확정
        if (stateTimer > 2f && IsAtLocalMaximum())
        {
            TransitionTo(MothState.SourceFound);
            Debug.Log("[나방] 누출원 도달! (국소 최대)");
            return;
        }

        // 수렴 → 최고 농도 지점으로 복귀
        if (stateTimer > 2f && HasConverged())
        {
            TransitionTo(MothState.ReturnToBest);
            Debug.Log($"[나방] Spiral 수렴 → 최고점 복귀");
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

    // ReturnToBest: 관측된 최고 농도 지점으로 물리적으로 이동 후 누출원 확정
    void HandleReturnToBest()
    {
        currentGoal = ValidateGoal(bestPos);

        Vector3 toBest = bestPos - transform.position;
        toBest.y = 0f;

        if (toBest.magnitude <= arrivalRadius)
        {
            // 도착 — 가스별 기준(dangerThreshold)으로 누출원 확정 여부 판단
            float confirmConc = plumeModel != null ? plumeModel.dangerThreshold : 40f;

            if (IsAtLocalMaximum() || bestConc >= confirmConc || returnCount >= maxReturnRetries)
            {
                TransitionTo(MothState.SourceFound);
                Debug.Log($"[나방] 최고점 도착 → 누출원 확정 (농도:{bestConc:F1}ppm, 재시도:{returnCount})");
            }
            else
            {
                // 아직 농도가 낮음 — 정점이 아닐 수 있으니 재탐색
                returnCount++;
                lastImproveTime = Time.time;
                TransitionTo(MothState.Surge);
                Debug.Log($"[나방] 최고점 농도 부족({bestConc:F1}<{confirmConc:F1}ppm) → 재탐색 {returnCount}/{maxReturnRetries}");
            }
            return;
        }

        // 복귀가 너무 오래 걸리면(장애물 등) 현 위치에서 확정
        if (stateTimer > 12f)
        {
            TransitionTo(MothState.SourceFound);
            Debug.Log("[나방] 복귀 시간 초과 → 현 위치에서 확정");
        }
    }

    // SourceFound: 누출원 도달 — 정지 + ROS 알림
    private float resultRepublishTimer = 0f;
    void HandleSourceFound()
    {
        currentGoal = transform.position;

        // 매 프레임 속도 0 발행 (로봇 완전 정지)
        PublishStopCommand();

        // 가스 완전 소실 → Idle
        if (stateTimer > 3f && !gasSensor.IsGasDetected(detectionThreshold))
        {
            searchStartTime = -1f;
            TransitionTo(MothState.Idle);
            Debug.Log("[나방] 가스 소실 → Idle 복귀 (재탐색 대기)");
            return;
        }

        // 결과 발행 — 2초 주기 재발행.
        // (일회성 발행은 프레임 스파이크/메시지 유실 시 영영 누락됨.
        //  patrol_navigator가 수신 후 모드를 바꾸면 MissionCoordinator가
        //  이 컴포넌트를 끄므로 재발행은 자동으로 멈춤 = ack 역할)
        resultRepublishTimer -= Time.deltaTime;
        if (resultRepublishTimer <= 0f)
        {
            resultRepublishTimer = 2f;

            string gasTypeName = plumeModel != null ? plumeModel.gasType.ToString() : "UNKNOWN";
            // 가스별 위험 판정 동봉 — patrol 쪽이 DANGER 알림을 놓쳐도 대피 가능
            string dangerFlag = (plumeModel != null && bestConc >= plumeModel.dangerThreshold)
                                ? "DANGER" : "OK";

            Debug.Log($"[나방] === 누출원 위치 특정 ===\n" +
                      $"  가스: {gasTypeName} ({dangerFlag})\n" +
                      $"  농도(최고): {bestConc:F1} ppm\n" +
                      $"  위치(최고점): ({bestPos.x:F1}, {bestPos.y:F1}, {bestPos.z:F1})");

            ros.Publish("/moth_search/result", new StringMsg
            {
                // 로봇 현재 위치가 아니라 '관측된 최고 농도 지점'을 누출원으로 보고
                data = $"SOURCE_FOUND|{gasTypeName}|" +
                       $"{bestConc:F1}|" +
                       $"{bestPos.x:F2},{bestPos.y:F2},{bestPos.z:F2}|" +
                       $"{dangerFlag}"
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
            total += allPlumes[i].GetConcentration(pos.x, pos.z);  // 2D — 센서와 동일 기준
        }
        return total;
    }

    // 국소 최대 검출: 현재 위치 농도가 주변(sampleDistance)보다 (거의) 높으면 = 농도 정점 = 누출원
    // 절대 ppm 기준이 아니라 "주변 대비 가장 높은 곳"으로 판정 (티칭 불필요).
    bool IsAtLocalMaximum()
    {
        Vector3 pos = transform.position;
        float here = SampleConcentrationAt(pos);

        // ★ 가스별 농도 게이트 — 누출원 근처(가파른 구역)에서만 정점 판정 허용.
        //   방출량 큰 가스(LNG)는 먼 곳 평평한 꼬리에서 eps가 기울기를 압도해
        //   감지 즉시 오판정 나는 것 방지.
        float minPeakConc = plumeModel != null
            ? plumeModel.dangerThreshold * 0.5f
            : detectionThreshold * 3f;
        if (here < Mathf.Max(detectionThreshold, minPeakConc)) return false;

        float maxNeighbor = 0f;
        for (int i = 0; i < sampleDirections; i++)
        {
            float angle = (360f / sampleDirections) * i * Mathf.Deg2Rad;
            Vector3 dir = new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
            float c = SampleConcentrationAt(pos + dir * sampleDistance);
            if (c > maxNeighbor) maxNeighbor = c;
        }
        // 여기가 주변 어디보다도 (여유 내) 높음 → 농도 정점
        // 여유는 절대값과 현재 농도의 8% 중 큰 쪽 (농도 스케일에 무관하게 동작)
        float eps = Mathf.Max(localMaxEpsilon, here * 0.08f);
        return here + eps >= maxNeighbor;
    }

    // 수렴 판정: 한동안 더 높은 농도를 못 찾음 = 정점 근처를 맴도는 중 → 최고점을 누출원으로 확정
    bool HasConverged()
    {
        return bestConc > detectionThreshold
               && Time.time - lastImproveTime > convergeSeconds;
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
                // Cast/ReturnToBest에서 복귀 시 수렴 시계 리셋 (복귀 첫 프레임 즉시 수렴 방지)
                lastImproveTime = Time.time;
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
            case MothState.SourceFound:
                resultRepublishTimer = 0f;   // 진입 즉시 첫 발행
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
            case MothState.ReturnToBest:
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, bestPos);
                Gizmos.DrawWireSphere(bestPos, arrivalRadius);
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
