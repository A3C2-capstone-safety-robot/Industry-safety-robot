// GasVisualizer.cs
using UnityEngine;

[RequireComponent(typeof(ParticleSystem))]
public class GasVisualizer : MonoBehaviour
{
    public GaussianPlumeModel plumeModel;
    private ParticleSystem ps;
    private ParticleSystem.EmissionModule emission;
    private ParticleSystem.MainModule mainModule;
    private ParticleSystem.VelocityOverLifetimeModule velocity;

    void Start()
    {
        ps = GetComponent<ParticleSystem>();
        emission = ps.emission;
        mainModule = ps.main;
        velocity = ps.velocityOverLifetime;

        emission.enabled = false;
        ApplyGasColor();
        ApplyGasVerticalMotion();
    }

    void Update()
    {
        if (plumeModel.isLeaking)
        {
            if (!ps.isPlaying) ps.Play();
            emission.enabled = true;

            // 누출 시간에 따라 파티클 양 증가
            float elapsed = Time.time - plumeModel.leakStartTime;
            emission.rateOverTime = Mathf.Lerp(10f, 80f, elapsed / 30f);
        }
        else
        {
            emission.enabled = false;
        }
    }

    // 가스 종류에 따라 파티클 색상 자동 설정
    void ApplyGasColor()
    {
        Color gasColor;

        switch (plumeModel.gasType)
        {
            case GasType.H2S:
                // 황화수소: 노란색 계열 (실제 황 성분)
                gasColor = new Color(0.9f, 0.8f, 0.2f, 0.3f);
                break;

            case GasType.LNG:
                // 메탄: 하얀색/연한 파란색 (무색이지만 시각화를 위해)
                gasColor = new Color(0.7f, 0.85f, 1f, 0.25f);
                break;

            case GasType.NH3:
                // 암모니아: 연한 초록색
                gasColor = new Color(0.5f, 0.9f, 0.5f, 0.3f);
                break;

            default:
                gasColor = new Color(0.8f, 0.8f, 0.8f, 0.3f);
                break;
        }

        mainModule.startColor = gasColor;
    }

    // 가스 종류에 따라 Y축 이동 방향 설정
    void ApplyGasVerticalMotion()
    {
        velocity.enabled = true;

        // 바람 방향 (XZ)
        velocity.x = plumeModel.windDirection.normalized.x * plumeModel.windSpeed * 0.3f;
        velocity.z = plumeModel.windDirection.normalized.y * plumeModel.windSpeed * 0.3f;

        // 가스별 Y축 거동
        velocity.y = plumeModel.verticalSpeed;
    }
}
