// VirtualGasSensor.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class VirtualGasSensor : MonoBehaviour
{
    [Header("센서 설정")]
    public float sensorRate = 5f;          // Hz (초당 샘플링 횟수)
    public float noiseLevel = 2f;          // 센서 노이즈 (ppm)

    [Header("ROS 토픽")]
    public string concentrationTopic = "/gas_concentration";
    public string alertTopic = "/gas_alert";

    private ROSConnection ros;
    private float timer = 0f;

    // 씬의 모든 가스 모델을 자동으로 찾음 (수동 연결 불필요)
    private GaussianPlumeModel[] allPlumeModels;

    // 현재 가장 높은 농도를 내는 누출원 (나방 알고리즘에서 참조)
    public GaussianPlumeModel DominantPlume { get; private set; }

    // 외부에서 읽을 수 있는 현재 농도 값 (나방 알고리즘에서 사용)
    public float CurrentConcentration { get; private set; }
    public bool IsAboveAlert { get; private set; }
    public bool IsAboveDanger { get; private set; }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32Msg>(concentrationTopic);
        ros.RegisterPublisher<StringMsg>(alertTopic);

        // 씬에 존재하는 모든 GaussianPlumeModel을 자동으로 찾기
        RefreshPlumeModels();
    }

    // 씬의 가스 모델 목록 갱신 (런타임 중 누출원이 추가될 수 있으므로)
    public void RefreshPlumeModels()
    {
        allPlumeModels = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        Debug.Log($"[가스 센서] 씬에서 {allPlumeModels.Length}개의 누출원 감지");
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / sensorRate) return;
        timer = 0f;

        // 모든 누출원의 농도를 합산 + 가장 높은 누출원 추적
        float totalConc = 0f;
        float maxConc = 0f;
        GaussianPlumeModel dominant = null;

        for (int i = 0; i < allPlumeModels.Length; i++)
        {
            var plume = allPlumeModels[i];
            if (plume == null || !plume.isLeaking) continue;

            float conc = plume.GetConcentration(
                transform.position.x,
                transform.position.y,
                transform.position.z
            );

            totalConc += conc;

            if (conc > maxConc)
            {
                maxConc = conc;
                dominant = plume;
            }
        }

        // 센서 노이즈 추가 (현실성)
        CurrentConcentration = Mathf.Max(0f, totalConc + Random.Range(-noiseLevel, noiseLevel));
        DominantPlume = dominant;

        // 경보 상태 업데이트
        if (dominant != null)
        {
            IsAboveAlert = CurrentConcentration >= dominant.alertThreshold;
            IsAboveDanger = CurrentConcentration >= dominant.dangerThreshold;
        }
        else
        {
            IsAboveAlert = false;
            IsAboveDanger = false;
        }

        // ROS 발행: 농도 값
        ros.Publish(concentrationTopic, new Float32Msg { data = CurrentConcentration });

        // 경보 발행
        if (IsAboveAlert && dominant != null)
        {
            string level = IsAboveDanger ? "DANGER" : "ALERT";
            ros.Publish(alertTopic, new StringMsg
            {
                data = $"[{level}] {dominant.gasType} 감지: {CurrentConcentration:F1} ppm @ " +
                       $"위치({transform.position.x:F1}, {transform.position.y:F1}, {transform.position.z:F1})"
            });
        }
    }

    // 가스 감지 여부 (나방 알고리즘에서 사용)
    public bool IsGasDetected(float minThreshold = 1f)
    {
        return CurrentConcentration >= minThreshold;
    }

    // 농도 변화율 (나방 알고리즘에서 방향 판단용)
    private float previousConcentration = 0f;
    public float GetConcentrationGradient()
    {
        float gradient = CurrentConcentration - previousConcentration;
        previousConcentration = CurrentConcentration;
        return gradient;
    }
}
