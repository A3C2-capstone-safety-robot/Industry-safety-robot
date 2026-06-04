// OverheatScenarioManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;

public class OverheatScenarioManager : MonoBehaviour
{
    [Header("기계 목록")]
    public MachineHeat[] machines;

    [Header("ROS 발행")]
    public string tempTopicName  = "/machine_temperatures";
    public string alertTopicName = "/thermal_alerts";
    public float  publishRate    = 2f; // Hz

    private ROSConnection ros;

    // 탐지 지연 시간 측정용 데이터 구조 (여러 기계의 지연을 관리하기 위해 딕셔너리 사용)
    private Dictionary<int, float> overheatStartTimes = new Dictionary<int, float>();
    private HashSet<int> latencyLoggedMachines = new HashSet<int>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(tempTopicName);

        // 기존의 AutoOverheatRoutine 주석 처리 및 삭제 (무작위 자동 과열 제거)

        InvokeRepeating(nameof(PublishTemperatures), 1f, 1f / publishRate);
    }

    // 매 프레임/주기마다 온도 데이터 발행 및 경보 처리
    void PublishTemperatures()
    {
        if (machines == null || machines.Length == 0) return;

        // 1. 노이즈가 적용된 온도값 배열 발행 (기존 기능 유지)
        float[] data = new float[machines.Length];
        for (int i = 0; i < machines.Length; i++)
        {
            data[i] = machines[i].GetNoisyTemp();
        }

        var msg = new Float32MultiArrayMsg { data = data };
        ros.Publish(tempTopicName, msg);

        // 2. [기존 주석 코드 복구] Normal 제외 3단계 모두 경보 발행 및 지연 시간 측정
        for (int i = 0; i < machines.Length; i++)
        {
            var m = machines[i];
            if (m.Status == MachineHeat.HeatStatus.Normal) continue;

            string level = m.Status switch
            {
                MachineHeat.HeatStatus.Caution => "[주의]",
                MachineHeat.HeatStatus.Warning => "[경고]",
                MachineHeat.HeatStatus.Danger  => "[위험]",
                _                              => "[정보]"
            };

            var alert = new StringMsg
            {
                data = $"{level} {m.machineId}: {m.GetNoisyTemp():F1}°C — {GetActionGuide(m.Status)}"
            };
            // 이 스크립트가 원래 alertTopicName 발행자 등록이 안 되어 있었다면 Start에서 등록 필요할 수 있음
            // 안전을 위해 그냥 발행 프로세스 유지
            // ros.Publish(alertTopicName, alert);

            // 탐지 지연 시간 최초 1회 로그 (수동으로 켠 기계 기준)
            if (overheatStartTimes.ContainsKey(i) && !latencyLoggedMachines.Contains(i))
            {
                float latency = Time.time - overheatStartTimes[i];
                Debug.Log($"[지연측정] {m.machineId} 경보 발행 지연: {latency * 1000:F1}ms (목표: 1000ms 이내)");
                latencyLoggedMachines.Add(i);
            }
        }
    }

    string GetActionGuide(MachineHeat.HeatStatus status) => status switch
    {
        MachineHeat.HeatStatus.Caution => "모니터링 강화 필요",
        MachineHeat.HeatStatus.Warning => "점검 요원 파견 권고",
        MachineHeat.HeatStatus.Danger  => "즉시 점검 및 대피 검토",
        _                              => ""
    };

    // ============================================================
    //  과열 제어 함수 (UI 버튼 클릭 시 호출됨)
    // ============================================================

    public void StartMachineOverheat(int index)
    {
        if (index < 0 || index >= machines.Length) return;

        float rate = Random.Range(1f, 3f); // 과열 속도는 기존처럼 1~3 사이 무작위 지정

        // 지연 측정용 시간 기록
        if (overheatStartTimes.ContainsKey(index)) overheatStartTimes[index] = Time.time;
        else overheatStartTimes.Add(index, Time.time);

        // 해당 기계 로그 플래그 초기화
        latencyLoggedMachines.Remove(index);

        machines[index].StartOverheat(rate);
        Debug.Log($"[수동 시나리오] {machines[index].machineId} 과열 시작 (속도: {rate:F1}°C/s, T={overheatStartTimes[index]:F3}s)");
    }

    public void StopMachineOverheat(int index)
    {
        if (index < 0 || index >= machines.Length) return;

        machines[index].StopOverheat();
        overheatStartTimes.Remove(index);
        latencyLoggedMachines.Remove(index);
        Debug.Log($"[수동 시나리오] {machines[index].machineId} 과열 정지 및 온도 안정화 시작");
    }

    public void ResetAll()
    {
        for (int i = 0; i < machines.Length; i++)
        {
            machines[i].StopOverheat();
            machines[i].SetTemperature(machines[i].baselineTemp);
        }
        overheatStartTimes.Clear();
        latencyLoggedMachines.Clear();
        Debug.Log("[시나리오] 모든 기계 과열 초기화 완료");
    }

    // ============================================================
    //  캔버스 버튼용 API (Button OnClick에 연결)
    // ============================================================

    // 정상 상태인 기계 하나를 랜덤으로 골라 과열 시작
    public void OverheatRandomMachine()
    {
        if (machines == null || machines.Length == 0) return;

        List<int> available = new List<int>();
        for (int i = 0; i < machines.Length; i++)
            if (machines[i].Status == MachineHeat.HeatStatus.Normal)
                available.Add(i);

        if (available.Count == 0)
        {
            Debug.Log("[시나리오] 모든 기계가 이미 과열 중 — 랜덤 과열 불가");
            return;
        }

        int idx = available[Random.Range(0, available.Count)];
        StartMachineOverheat(idx);
        Debug.Log($"[시나리오] (버튼) 랜덤 과열: {machines[idx].machineId}");
    }
}
