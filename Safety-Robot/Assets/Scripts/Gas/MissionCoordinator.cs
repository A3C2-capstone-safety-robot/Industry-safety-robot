using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// 임무 모드 코디네이터.
/// patrol_navigator(ROS2)가 발행하는 /robot_mode 를 구독해서,
/// "GAS_TRACKING" 일 때만 MothSearch(가스 추적)를 켠다.
/// 그 외(PATROL, EVACUATING)에는 MothSearch 를 끈다.
/// → /cmd_vel 운전수가 항상 한 명만 되도록 보장 (순찰 Nav2 ↔ MothSearch).
/// 씬의 아무 빈 오브젝트(또는 로봇)에 붙여서 사용.
/// </summary>
public class MissionCoordinator : MonoBehaviour
{
    [Header("ROS")]
    public string modeTopic = "/robot_mode";

    [Header("가스 추적 알고리즘")]
    [Tooltip("MothSearchAlgorithm 컴포넌트를 연결. GAS_TRACKING 동안에만 켜짐")]
    public MothSearchAlgorithm mothSearch;

    private ROSConnection ros;
    private string pendingMode = null;
    private readonly object _lock = new object();

    void Start()
    {
        // 평소엔 MothSearch OFF (순찰 중 /cmd_vel 충돌 방지)
        if (mothSearch != null)
            mothSearch.enabled = false;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(modeTopic, OnMode);

        Debug.Log("[MissionCoordinator] 준비됨 — /robot_mode 구독 중");
    }

    // ROS 콜백(다른 스레드일 수 있음) → 값만 저장하고 메인스레드(Update)에서 적용
    void OnMode(StringMsg msg)
    {
        lock (_lock) { pendingMode = msg.data; }
    }

    void Update()
    {
        string mode;
        lock (_lock)
        {
            if (pendingMode == null) return;
            mode = pendingMode;
            pendingMode = null;
        }

        bool gasTracking = (mode == "GAS_TRACKING");
        if (mothSearch != null && mothSearch.enabled != gasTracking)
        {
            mothSearch.enabled = gasTracking;
            Debug.Log(gasTracking
                ? "[MissionCoordinator] 가스 추적 모드 ON — MothSearch 활성화"
                : "[MissionCoordinator] " + mode + " — MothSearch 비활성화");
        }
    }
}
