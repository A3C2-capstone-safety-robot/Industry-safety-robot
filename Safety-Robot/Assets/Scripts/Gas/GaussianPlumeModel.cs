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
                // 황화수소: 무거움 → 바닥으로 가라앉음, 느리게 확산
                emissionRate = 80f;
                verticalSpeed = -0.3f;      // 하강
                verticalSigma = 0.2f;       // Y축 확산 좁음 (바닥에 깔림)
                sigmaBase = 0.4f;           // 초기 확산 좁음
                sigmaGrowth = 0.03f;        // 느리게 퍼짐
                alertThreshold = 10f;       // 10 ppm (냄새 감지)
                dangerThreshold = 100f;     // 100 ppm (IDLH)
                break;

            case GasType.LNG:
                // 메탄: 가벼움 → 위로 상승, 빠르게 확산
                emissionRate = 120f;
                verticalSpeed = 0.5f;       // 상승
                verticalSigma = 0.5f;       // Y축 넓게 퍼짐
                sigmaBase = 0.6f;           // 초기 확산 넓음
                sigmaGrowth = 0.08f;        // 빠르게 퍼짐
                alertThreshold = 5000f;     // LEL 10% (50000 ppm의 10%)
                dangerThreshold = 25000f;   // LEL 50%
                break;

            case GasType.NH3:
                // 암모니아: 약간 가벼움 → 살짝 상승, 중간 속도 확산
                emissionRate = 100f;
                verticalSpeed = 0.15f;      // 약간 상승
                verticalSigma = 0.4f;       // 중간 Y축 확산
                sigmaBase = 0.5f;           // 중간 초기 확산
                sigmaGrowth = 0.05f;        // 중간 속도
                alertThreshold = 25f;       // 25 ppm (냄새 감지)
                dangerThreshold = 300f;     // 300 ppm (IDLH)
                break;
        }
    }

    // 3D 농도 계산 (Y축 포함)
    public float GetConcentration(float x, float y, float z)
    {
        if (!isLeaking || leakSource == null) return 0f;

        float elapsed = Time.time - leakStartTime;
        if (elapsed <= 0f) return 0f;

        // 가스 구름 중심: XZ 평면에서 바람 방향으로 이동
        Vector2 windNorm = windDirection.normalized;
        float cloudX = leakSource.position.x + windNorm.x * windSpeed * elapsed;
        float cloudZ = leakSource.position.z + windNorm.y * windSpeed * elapsed;

        // Y축: 가스 종류에 따라 상승 또는 하강
        float cloudY = leakSource.position.y + verticalSpeed * elapsed;

        // 시간에 따라 확산 범위 증가
        float sigma = sigmaBase + sigmaGrowth * elapsed;
        float sigmaY = verticalSigma + sigmaGrowth * elapsed * 0.5f;

        // 3D 가우시안 분포
        float distXZ = (x - cloudX) * (x - cloudX) + (z - cloudZ) * (z - cloudZ);
        float distY = (y - cloudY) * (y - cloudY);

        float concXZ = Mathf.Exp(-distXZ / (2f * sigma * sigma));
        float concY = Mathf.Exp(-distY / (2f * sigmaY * sigmaY));

        float concentration = emissionRate * concXZ * concY;

        return Mathf.Max(0f, concentration);
    }

    // 2D 버전 (기존 호환 — Y축 무시)
    public float GetConcentration(float x, float z)
    {
        if (!isLeaking || leakSource == null) return 0f;

        float elapsed = Time.time - leakStartTime;
        if (elapsed <= 0f) return 0f;

        Vector2 windNorm = windDirection.normalized;
        float cloudX = leakSource.position.x + windNorm.x * windSpeed * elapsed;
        float cloudZ = leakSource.position.z + windNorm.y * windSpeed * elapsed;

        float sigma = sigmaBase + sigmaGrowth * elapsed;
        float distSq = (x - cloudX) * (x - cloudX) + (z - cloudZ) * (z - cloudZ);
        float concentration = emissionRate * Mathf.Exp(-distSq / (2f * sigma * sigma));

        return Mathf.Max(0f, concentration);
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

        float elapsed = Time.time - leakStartTime;
        Vector2 windNorm = windDirection.normalized;
        float voidX = leakSource.position.x + windNorm.x * windSpeed * elapsed * 0.5f;
        float voidZ = leakSource.position.z + windNorm.y * windSpeed * elapsed * 0.5f;
        float sigmaV = 0.3f + 0.03f * elapsed;
        float voidDistSq = (x - voidX) * (x - voidX) + (z - voidZ) * (z - voidZ);
        float voidVal = 60f * Mathf.Exp(-voidDistSq / (2f * sigmaV * sigmaV));

        return Mathf.Max(0f, main - voidVal * 0.7f);
    }

    // Inspector에서 가스 타입 변경 시 자동 반영
    void OnValidate()
    {
        ApplyGasPreset(gasType);
    }
}
