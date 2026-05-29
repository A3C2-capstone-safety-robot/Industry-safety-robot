// OverheatScenarioManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections;
using System.Linq;

public class OverheatScenarioManager : MonoBehaviour
{
    [Header("기계 목록")]
    public MachineHeat[] machines;

    [Header("자동 시나리오")]
    public bool  autoTrigger     = true;
    public float triggerInterval = 30f;

    [Header("ROS 발행")]
    public string tempTopicName  = "/machine_temperatures";
    public string alertTopicName = "/thermal_alerts";
    public float  publishRate    = 2f; // Hz

    private ROSConnection ros;

    // 탐지 지연 시간 측정용
    private float overheatStartTime = -1f;
    private int   overheatMachineIdx = -1;
    private bool  latencyLogged = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(tempTopicName);
        
        if (autoTrigger)
            StartCoroutine(AutoOverheatRoutine());

        InvokeRepeating(nameof(PublishTemperatures), 1f, 1f / publishRate);
    }
    

    IEnumerator AutoOverheatRoutine()
    {
        while (true)
        {
            yield return new WaitForSeconds(triggerInterval);

            foreach (var m in machines) m.StopOverheat();

            int   idx  = Random.Range(0, machines.Length);
            float rate = Random.Range(1f, 3f);

            // 과열 시작 시각 기록 (지연 측정용)
            overheatStartTime  = Time.time;
            overheatMachineIdx = idx;
            latencyLogged      = false;

            machines[idx].StartOverheat(rate);
            Debug.Log($"[시나리오] {machines[idx].machineId} 과열 시작 " +
                      $"(속도: {rate:F1}°C/s, T={overheatStartTime:F3}s)");
        }
    }

    // PublishTemperatures() 수정
    void PublishTemperatures()
    {
        if (machines == null || machines.Length == 0) return;

        float[] data = new float[machines.Length];
        for (int i = 0; i < machines.Length; i++)
        {
            data[i] = machines[i].GetNoisyTemp();
        }

        var msg = new Float32MultiArrayMsg { data = data };
        ros.Publish(tempTopicName, msg);
    }

    // void PublishTemperatures()
    // {
    //     // 노이즈가 적용된 온도값 발행
    //     var msg = new Float32MultiArrayMsg();
    //     msg.data = machines.Select(m => m.GetNoisyTemp()).ToArray();
    //     ros.Publish(tempTopicName, msg);

    //     // Normal 제외 3단계 모두 경보 발행
    //     foreach (var m in machines)
    //     {
    //         if (m.Status == MachineHeat.HeatStatus.Normal) continue;

    //         string level = m.Status switch
    //         {
    //             MachineHeat.HeatStatus.Caution => "[주의]",
    //             MachineHeat.HeatStatus.Warning => "[경고]",
    //             MachineHeat.HeatStatus.Danger  => "[위험]",
    //             _                              => "[정보]"
    //         };

    //         var alert = new StringMsg
    //         {
    //             data = $"{level} {m.machineId}: {m.GetNoisyTemp():F1}°C — {GetActionGuide(m.Status)}"
    //         };
    //         ros.Publish(alertTopicName, alert);

    //         // 탐지 지연 시간 최초 1회 로그
    //         if (!latencyLogged && overheatStartTime >= 0 &&
    //             System.Array.IndexOf(machines, m) == overheatMachineIdx)
    //         {
    //             float latency = Time.time - overheatStartTime;
    //             Debug.Log($"[지연측정] {m.machineId} 경보 발행 지연: {latency * 1000:F1}ms " +
    //                       $"(목표: 1000ms 이내)");
    //             latencyLogged = true;
    //         }
    //     }
    // }

    string GetActionGuide(MachineHeat.HeatStatus status) => status switch
    {
        MachineHeat.HeatStatus.Caution => "모니터링 강화 필요",
        MachineHeat.HeatStatus.Warning => "점검 요원 파견 권고",
        MachineHeat.HeatStatus.Danger  => "즉시 점검 및 대피 검토",
        _                              => ""
    };

    public void ManualSetTemp(int machineIndex, float temp)
    {
        if (machineIndex < machines.Length)
            machines[machineIndex].SetTemperature(temp);
    }

    public void ResetAll()
    {
        foreach (var m in machines)
        {
            m.StopOverheat();
            m.SetTemperature(m.baselineTemp);
        }
    }

    [ContextMenu("강제 무작위 과열 발생")]
    public void TriggerRandomOverheat()
    {
        foreach (var m in machines) m.StopOverheat();

        int idx = Random.Range(0, machines.Length);
        float rate = Random.Range(1f, 3f);

        overheatStartTime = Time.time;
        overheatMachineIdx = idx;
        latencyLogged = false;

        machines[idx].StartOverheat(rate);
        Debug.Log($"[수동 시나리오] {machines[idx].machineId} 과열 시작 (속도: {rate:F1}°C/s)");
    }
}
