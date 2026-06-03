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

    [Header("UI 설정")]
    public bool showUI = true;
    public KeyCode toggleUIKey = KeyCode.H;        // UI 표시/숨김 토글 키 (히트 = H)

    [Header("ROS 발행")]
    public string tempTopicName  = "/machine_temperatures";
    public string alertTopicName = "/thermal_alerts";
    public float  publishRate    = 2f; // Hz

    private ROSConnection ros;

    // 탐지 지연 시간 측정용 데이터 구조 (여러 기계의 지연을 관리하기 위해 딕셔너리 사용)
    private Dictionary<int, float> overheatStartTimes = new Dictionary<int, float>();
    private HashSet<int> latencyLoggedMachines = new HashSet<int>();

    // UI 관련
    private bool uiVisible = true;
    private Vector2 scrollPosition;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(tempTopicName);

        // 기존의 AutoOverheatRoutine 주석 처리 및 삭제 (무작위 자동 과열 제거)

        InvokeRepeating(nameof(PublishTemperatures), 1f, 1f / publishRate);
    }

    void Update()
    {
        // H 키로 UI 켜고 끄기
        if (Input.GetKeyDown(toggleUIKey))
        {
            uiVisible = !uiVisible;
        }
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
    //  개발자용 OnGUI 화면 버튼 구현
    // ============================================================
    void OnGUI()
    {
        if (!showUI || !uiVisible || machines == null) return;

        float panelWidth = 300f;
        float panelX = 20f; // 가스 누출 매니저와 겹치지 않게 왼쪽 상단(20)에 배치
        float panelY = 20f;

        // 배경 패널 크기 계산
        float panelHeight = Mathf.Min(140 + machines.Length * 32, 500);
        GUI.Box(new Rect(panelX, panelY, panelWidth, panelHeight), "");

        GUILayout.BeginArea(new Rect(panelX + 10, panelY + 10, panelWidth - 20, panelHeight - 20));

        // 제목
        GUIStyle titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter
        };
        GUILayout.Label("Machine Overheat Control", titleStyle);
        GUILayout.Space(10);

        // 전체 초기화 버튼
        GUI.backgroundColor = new Color(0.4f, 0.6f, 1f); // 파란색 계열
        if (GUILayout.Button("Reset All Machines", GUILayout.Height(30)))
        {
            ResetAll();
        }
        GUI.backgroundColor = Color.white;
        GUILayout.Space(10);

        // 각 머신별 스크롤 뷰 및 버튼 리스트
        scrollPosition = GUILayout.BeginScrollView(scrollPosition, GUILayout.Height(320));

        for (int i = 0; i < machines.Length; i++)
        {
            var m = machines[i];
            string machineName = m.machineId;

            // 현재 과열 중인지 여부 판단 (임의로 온도가 baseline보다 높거나 Status가 Normal이 아니면 과열 상태 표시)
            bool isOverheating = m.Status != MachineHeat.HeatStatus.Normal;

            GUILayout.BeginHorizontal();

            // 상태 표시 점 (과열중이면 빨간색, 정상 성향이면 초록색)
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label)
            {
                fontStyle = FontStyle.Bold,
                normal = { textColor = isOverheating ? Color.red : Color.green }
            };
            GUILayout.Label(isOverheating ? "●" : "○", statusStyle, GUILayout.Width(15));

            // 머신 이름과 현재 온도 표시
            GUILayout.Label($"{machineName} ({m.GetNoisyTemp():F1}°C)", GUILayout.Width(145));

            // Start / Stop 버튼 생성
            if (isOverheating)
            {
                GUI.backgroundColor = new Color(1f, 0.6f, 0.6f); // 연빨강
                if (GUILayout.Button("Cool", GUILayout.Width(65))) // 과열 식히기
                {
                    StopMachineOverheat(i);
                }
            }
            else
            {
                GUI.backgroundColor = new Color(1f, 0.8f, 0.4f); // 주황/노랑 계열
                if (GUILayout.Button("Heat", GUILayout.Width(65))) // 과열 시작
                {
                    StartMachineOverheat(i);
                }
            }
            GUI.backgroundColor = Color.white;

            GUILayout.EndHorizontal();
            GUILayout.Space(4);
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }
}
