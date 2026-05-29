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
    public float sigmaBase = 0.5f;         // 초기 확산 반경
    public float sigmaGrowth = 0.05f;      // 시간에 따른 확산 증가율

    [Header("Y축 거동 (자동 설정됨)")]
    public float verticalSpeed = 0f;       // + 상승, - 하강 (m/s)
    public float verticalSigma = 0.3f;     // Y축 확산 범위

    [Header("경보 임계값 (자동 설정됨)")]
    public float alertThreshold = 30f;     // ppm
    public float dangerThreshold = 100f;   // ppm (IDLH)

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
                // 황화수소: 무거움 → 바닥으로 가라앉음
                emissionRate = 100f;        // 누출원 바로 옆 최대 농도 (ppm)
                verticalSpeed = -0.3f;      // 하강
                verticalSigma = 1.5f;       // Y축 확산
                sigmaBase = 5f;             // 초기 확산 반경 (m) — 5m 밖부터 급감
                sigmaGrowth = 0.3f;         // 시간에 따라 반경 증가 (m/s)
                alertThreshold = 10f;       // 10 ppm
                dangerThreshold = 50f;      // 50 ppm
                break;

            case GasType.LNG:
                // 메탄: 가벼움 → 위로 상승, 빠르게 확산
                emissionRate = 120f;
                verticalSpeed = 0.5f;       // 상승
                verticalSigma = 2f;         // Y축 넓게
                sigmaBase = 6f;             // 넓은 확산 반경
                sigmaGrowth = 0.4f;         // 빠르게 퍼짐
                alertThreshold = 5000f;     // LEL 10%
                dangerThreshold = 25000f;   // LEL 50%
                break;

            case GasType.NH3:
                // 암모니아: 약간 가벼움 → 살짝 상승
                emissionRate = 110f;
                verticalSpeed = 0.15f;      // 약간 상승
                verticalSigma = 1.8f;       // 중간 Y축 확산
                sigmaBase = 5.5f;           // 중간 확산 반경
                sigmaGrowth = 0.35f;        // 중간 속도
                alertThreshold = 25f;       // 25 ppm
                dangerThreshold = 150f;     // 150 ppm
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

        // 시간에 따른 플룸 성장 (5초에 걸쳐 서서히 확대)
        float timeFactor = Mathf.Clamp01(elapsed / 5f);

        // 확산 반경: 시간에 따라 천천히 커짐
        float sigma = sigmaBase + sigmaGrowth * Mathf.Min(elapsed, 20f);

        // ★ 핵심: 거리에 따른 지수 감쇠 (가까울수록 높고, 멀수록 급격히 감소)
        float distConc = emissionRate * Mathf.Exp(-dist * dist / (2f * sigma * sigma));

        // 바람 방향 보너스 (풍하 쪽이 약간 더 높음)
        Vector2 windNorm = windDirection.normalized;
        float downwind = dx * windNorm.x + dz * windNorm.y;
        float windBonus = 1f + Mathf.Max(0f, downwind / (sigma + 1f)) * 0.3f;

        // Y축 가우시안 (가스 수직 거동)
        float sigmaY = verticalSigma + sigmaGrowth * elapsed * 0.3f;
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

        float timeFactor = Mathf.Clamp01(elapsed / 5f);
        float sigma = sigmaBase + sigmaGrowth * Mathf.Min(elapsed, 20f);

        // 거리에 따른 지수 감쇠
        float distConc = emissionRate * Mathf.Exp(-dist * dist / (2f * sigma * sigma));

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
