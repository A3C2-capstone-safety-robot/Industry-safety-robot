// GaussianPlumeModel.cs
using UnityEngine;

public enum GasType
{
    H2S,        // 황화수소 — 공기보다 무거움 (1.19), 바닥으로 가라앉음
    LNG,        // 메탄(CH4) — 공기보다 가벼움 (0.55), 위로 상승
    NH3         // 암모니아 — 공기보다 약간 가벼움 (0.73), 중간 높이로 퍼짐
}

public class GaussianPlumeModel : MonoBehaviour
{
    [Header("가스 종류")]
    public GasType gasType = GasType.H2S;

    [Header("누출원 설정")]
    public Transform leakSource;           // 누출 지점 오브젝트
    public float emissionRate = 100f;      // 가스 방출량 (ppm 기준)

    [Header("바람 설정")]
    public Vector2 windDirection = new Vector2(1f, 0f); // x, z 평면 바람 방향
    public float windSpeed = 1.5f;         // m/s

    [Header("확산 파라미터")]
    public float sigmaBase = 0.5f;         // 누출원에서의 확산 반경 (m)
    public float sigmaGrowth = 0.3f;       // 거리당 확산 증가율 (m/m) — 현실 플룸: σ는 거리의 함수

    [Header("Y축 거동 (자동 설정됨)")]
    public float verticalSpeed = 0f;       // + 상승, - 하강 (m/s)
    public float verticalSigma = 0.3f;     // Y축 확산 범위

    [Header("경보 임계값 (자동 설정됨)")]
    public float alertThreshold = 30f;     // ppm
    public float dangerThreshold = 100f;   // ppm (IDLH)

    [Header("누출 속도 (자동 설정됨)")]
    public float buildupTime = 30f;        // 최대 농도까지 걸리는 시간(초) — 클수록 천천히 누출

    [Header("시뮬레이션")]
    public float leakStartTime = -1f;
    public bool isLeaking = false;

    void Awake()
    {
        ApplyGasPreset(gasType);
    }

    // 가스 종류에 따라 파라미터 자동 세팅
    public void ApplyGasPreset(GasType type)
    {
        gasType = type;

        switch (type)
        {
            case GasType.H2S:
                // 황화수소: 무거움 → 바닥으로 가라앉음. 제일 독성 → 낮은 농도서 위험.
                emissionRate = 60f;         // 피크 농도 — IDLH 언저리(현실적)
                verticalSpeed = -0.3f;      // 하강
                verticalSigma = 1.5f;       // Y축 확산
                sigmaBase = 2f;             // 누출원 확산 반경 (m) — 좁게(국소)
                sigmaGrowth = 0.3f;         // 거리당 퍼짐 — 근처는 뾰족, 멀리는 완만
                alertThreshold = 3f;        // 감지(추적 시작) — 사실상 검출 즉시 (노이즈 2 위)
                dangerThreshold = 20f;      // 위험·대피 (고경보 수준 — IDLH 전에 대피)
                buildupTime = 40f;          // 무거워서 천천히 축적
                break;

            case GasType.LNG:
                // 메탄: 가벼움 → 위로 상승, 빠르게 확산. 폭발성(ppm당 독성 낮음) → 높은 기준.
                emissionRate = 300f;        // 피크 농도
                verticalSpeed = 0.5f;       // 상승
                verticalSigma = 2f;         // Y축 넓게
                sigmaBase = 3f;             // 가벼워 약간 더 넓게(그래도 국소)
                sigmaGrowth = 0.4f;         // 가벼운 가스 — 더 빨리 퍼짐
                alertThreshold = 3f;        // 감지(추적 시작) — 사실상 검출 즉시
                dangerThreshold = 100f;     // 위험·대피 (고경보 수준, ~20% LEL 비례)
                buildupTime = 25f;          // 가벼워 비교적 빨리 퍼짐
                break;

            case GasType.NH3:
                // 암모니아: 약간 가벼움 → 살짝 상승. 중간 위험도.
                emissionRate = 150f;        // 피크 농도 (IDLH 300 미만)
                verticalSpeed = 0.15f;      // 약간 상승
                verticalSigma = 1.8f;       // 중간 Y축 확산
                sigmaBase = 2.5f;           // 중간(국소)
                sigmaGrowth = 0.35f;        // 중간 퍼짐
                alertThreshold = 3f;        // 감지(추적 시작) — 사실상 검출 즉시
                dangerThreshold = 50f;      // 위험·대피 (고경보 수준)
                buildupTime = 30f;          // 중간 속도로 누출
                break;
        }
    }

    // 3D 농도 계산 — 연속 방출 플룸 모델
    // 누출원에서 가까울수록 농도가 높고, 멀어질수록 급격히 감소
    public float GetConcentration(float x, float y, float z)
    {
        if (!isLeaking || leakSource == null) return 0f;

        float elapsed = Time.time - leakStartTime;
        if (elapsed <= 0f) return 0f;

        Vector3 src = leakSource.position;

        // 누출원에서의 직선 거리
        float dx = x - src.x;
        float dz = z - src.z;
        float dist = Mathf.Sqrt(dx * dx + dz * dz);

        // 농도 축적 (buildupTime에 걸쳐 준정상상태 도달)
        float timeFactor = Mathf.Clamp01(elapsed / Mathf.Max(0.1f, buildupTime));

        // ★ 현실 플룸: σ는 시간이 아니라 '누출원에서의 거리'의 함수 (Pasquill-Gifford 근사)
        //   근처는 좁고 뾰족(정밀 위치 특정), 멀리는 넓고 완만(원거리 감지)
        float sigma = sigmaBase + sigmaGrowth * dist;

        // 거리 감쇠 × 질량 보존 항(sigmaBase/sigma) — σ가 커진 만큼 희석
        float distConc = emissionRate * (sigmaBase / sigma)
                         * Mathf.Exp(-dist * dist / (2f * sigma * sigma));

        // 바람 방향 보너스 (풍하 쪽이 약간 더 높음)
        Vector2 windNorm = windDirection.normalized;
        float downwind = dx * windNorm.x + dz * windNorm.y;
        float windBonus = 1f + Mathf.Max(0f, downwind / (sigma + 1f)) * 0.3f;

        // Y축 가우시안 (가스 수직 거동) — 수직 퍼짐도 거리 비례
        float sigmaY = verticalSigma + sigmaGrowth * dist * 0.5f;
        float cloudY = src.y + verticalSpeed * Mathf.Min(elapsed, 15f);
        float distY = (y - cloudY) * (y - cloudY);
        float vertConc = Mathf.Exp(-distY / (2f * sigmaY * sigmaY));

        return Mathf.Max(0f, distConc * windBonus * vertConc * timeFactor);
    }

    // 2D 버전 (기존 호환 — Y축 무시)
    public float GetConcentration(float x, float z)
    {
        if (!isLeaking || leakSource == null) return 0f;

        float elapsed = Time.time - leakStartTime;
        if (elapsed <= 0f) return 0f;

        float dx = x - leakSource.position.x;
        float dz = z - leakSource.position.z;
        float dist = Mathf.Sqrt(dx * dx + dz * dz);

        float timeFactor = Mathf.Clamp01(elapsed / Mathf.Max(0.1f, buildupTime));

        // 거리 기반 σ + 질량 보존 (3D 버전과 동일 기준)
        float sigma = sigmaBase + sigmaGrowth * dist;
        float distConc = emissionRate * (sigmaBase / sigma)
                         * Mathf.Exp(-dist * dist / (2f * sigma * sigma));

        // 바람 보너스
        Vector2 windNorm = windDirection.normalized;
        float downwind = dx * windNorm.x + dz * windNorm.y;
        float windBonus = 1f + Mathf.Max(0f, downwind / (sigma + 1f)) * 0.3f;

        return Mathf.Max(0f, distConc * windBonus * timeFactor);
    }

    // 누출 시작
    public void StartLeak()
    {
        isLeaking = true;
        leakStartTime = Time.time;
        Debug.Log($"[가스 누출] {gasType} — {leakSource.name} 위치에서 누출 시작!");
    }

    // 누출 중지
    public void StopLeak()
    {
        isLeaking = false;
        Debug.Log($"[가스 누출] {gasType} — 누출 중지");
    }

    // 바람에 의한 공백(void) 포함 버전 — 나방 알고리즘 테스트용
    public float GetConcentrationWithVoid(float x, float z)
    {
        if (!isLeaking) return 0f;

        float main = GetConcentration(x, z);

        // 누출원 풍하 쪽에 난류 공백 영역 생성
        float elapsed = Time.time - leakStartTime;
        Vector2 windNorm = windDirection.normalized;
        float voidDist = Mathf.Min(windSpeed * elapsed * 0.3f, 5f); // 최대 5m
        float voidX = leakSource.position.x + windNorm.x * voidDist;
        float voidZ = leakSource.position.z + windNorm.y * voidDist;
        float sigmaV = 0.3f + 0.03f * elapsed;
        float voidDistSq = (x - voidX) * (x - voidX) + (z - voidZ) * (z - voidZ);
        float voidVal = 60f * Mathf.Exp(-voidDistSq / (2f * sigmaV * sigmaV));

        return Mathf.Max(0f, main - voidVal * 0.7f);
    }

    // Inspector에서 가스 타입 변경 시 자동 반영
    // 수동으로 값을 조정하고 싶으면 이 함수를 비활성화하세요
    private GasType lastGasType;
    void OnValidate()
    {
        // 가스 타입이 변경됐을 때만 프리셋 적용 (수동 조정 보호)
        if (lastGasType != gasType)
        {
            lastGasType = gasType;
            ApplyGasPreset(gasType);
        }
    }
}
