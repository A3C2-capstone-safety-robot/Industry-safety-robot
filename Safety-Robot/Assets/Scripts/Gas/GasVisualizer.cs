// GasVisualizer.cs
// Inspector에서 세팅한 Particle System 값을 존중합니다.
// 코드는 emission on/off, 색상, Y축 방향만 제어합니다.
using UnityEngine;

[RequireComponent(typeof(ParticleSystem))]
public class GasVisualizer : MonoBehaviour
{
    public GaussianPlumeModel plumeModel;

    [Header("Y축 스케일 (Inspector에서 조절)")]
    public float verticalScale = 0.3f;      // Y축 속도 스케일 (줄이면 덜 올라감)

    private ParticleSystem ps;
    private ParticleSystem.EmissionModule emission;
    private ParticleSystem.MainModule mainModule;
    private ParticleSystem.VelocityOverLifetimeModule velocity;

    // Inspector에서 설정한 원래 emission rate 저장
    private float originalEmissionRate;

    void Start()
    {
        ps = GetComponent<ParticleSystem>();
        emission = ps.emission;
        mainModule = ps.main;
        velocity = ps.velocityOverLifetime;

        // Inspector에서 세팅한 emission rate 기억
        originalEmissionRate = emission.rateOverTime.constant;

        // 시작 시 꺼두기 (누출 시작 전)
        emission.enabled = false;

        // 색상과 Y축 방향만 코드에서 설정
        ApplyGasColor();
        ApplyGasVerticalMotion();

        // ★ 나머지 (startLifetime, startSpeed, startSize, Noise 등)는
        //   Inspector에서 세팅한 값 그대로 사용!
    }

    void Update()
    {
        if (plumeModel == null) return;

        if (plumeModel.isLeaking)
        {
            if (!ps.isPlaying) ps.Play();
            emission.enabled = true;
            // Inspector에서 설정한 emission rate 그대로 사용
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
                gasColor = new Color(0.9f, 0.8f, 0.2f, 0.3f);
                break;
            case GasType.LNG:
                gasColor = new Color(0.7f, 0.85f, 1f, 0.25f);
                break;
            case GasType.NH3:
                gasColor = new Color(0.5f, 0.9f, 0.5f, 0.3f);
                break;
            default:
                gasColor = new Color(0.8f, 0.8f, 0.8f, 0.3f);
                break;
        }

        mainModule.startColor = gasColor;
    }

    // 가스 종류에 따라 Y축 방향만 설정
    // Inspector에서 Velocity over Lifetime이 꺼져 있으면 건드리지 않음
    void ApplyGasVerticalMotion()
    {
        // Inspector에서 이미 꺼놨으면 코드가 강제로 켜지 않음
        if (!velocity.enabled) return;

        // 켜져 있을 때만 Y축 조정
        float ySpeed = plumeModel.verticalSpeed * verticalScale;
        velocity.y = new ParticleSystem.MinMaxCurve(ySpeed * 0.5f, ySpeed);
    }
}
