// MachinePositionPublisher.cs  (v3 — 시야 가림(occlusion) 검사 추가)
// 설비 중심 좌표(3) + 바운딩박스 반크기 extents(3) = 설비당 6개 float 발행.
// 열화상 카메라에서 설비 중심으로 레이캐스트해서 벽/다른 설비에 가려진 기계는
// 6개 값을 전부 0으로 발행 → ros2 thermal_visualizer가 "안 보임"으로 처리.
// 포맷: [cx0,cy0,cz0, ex0,ey0,ez0,  cx1,cy1,cz1, ex1,ey1,ez1, ...]
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class MachinePositionPublisher : MonoBehaviour
{
    [Header("설비 목록 (OverheatScenarioManager와 동일한 순서로 연결!)")]
    public MachineHeat[] machines;

    [Header("열화상 카메라 (가림 검사 시점)")]
    [Tooltip("반드시 로봇에 달린 카메라(CameraPublisher의 robotCamera와 동일)를 연결. " +
             "비워두면 Camera.main을 쓰는데, 그건 관전 카메라라 가림 판정이 엉터리가 됨!")]
    public Camera thermalCamera;

    [Header("가림 검사에서 무시할 레이어 (로봇 자신 등)")]
    [Tooltip("카메라가 로봇 몸체 안/근처에 있으면 로봇 콜라이더에 막혀 전부 '가림' 판정됨. " +
             "로봇 레이어를 여기에 지정해서 제외할 것")]
    public LayerMask ignoreLayers;

    [Header("ROS 설정")]
    public string topicName = "/machine_world_positions";
    public string frameId = "map";
    public float publishRate = 1f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);

        if (thermalCamera == null)
        {
            thermalCamera = Camera.main;
            Debug.LogWarning("[MachinePositionPublisher] thermalCamera 미할당 → Camera.main 사용 중. " +
                             "가림 판정이 로봇 시점이 아니라 관전 시점으로 계산됩니다! " +
                             "Inspector에서 로봇 카메라를 연결하세요.", this);
        }

        InvokeRepeating(nameof(PublishPositions), 1f, 1f / publishRate);
    }

    void PublishPositions()
    {
        if (machines == null || machines.Length == 0 || thermalCamera == null) return;

        // 설비당 6 floats: [center(3), extents(3)]
        // Unity(Left-handed) → ROS(Right-handed) 축 변환
        //   ROS x =  Unity z / ROS y = -Unity x / ROS z =  Unity y
        float[] data = new float[machines.Length * 6];
        Vector3 cameraPos = thermalCamera.transform.position;

        for (int i = 0; i < machines.Length; i++)
        {
            if (machines[i] == null) continue;

            Bounds bounds = GetWorldBounds(machines[i]);
            Vector3 center = bounds.center;
            Vector3 e = bounds.extents;

            // ── 시야 가림 검사: 카메라→설비 중심 레이캐스트 ──
            // 처음 맞은 것이 자기 자신(또는 자식)이 아니면 = 다른 물체가 가로막음
            Vector3 direction = center - cameraPos;
            float distance = direction.magnitude;

            if (distance > 0.01f
                && Physics.Raycast(cameraPos, direction / distance, out RaycastHit hit,
                                   distance + 0.5f, ~ignoreLayers,
                                   QueryTriggerInteraction.Ignore))
            {
                bool isSelf = hit.transform == machines[i].transform
                              || hit.transform.IsChildOf(machines[i].transform);
                if (!isSelf)
                {
                    // 가려짐 → 0으로 채워서 "안 보임" 표시 (data는 기본값 0이라 그대로 둠)
                    continue;
                }
            }

            // ── 보이는 경우에만 실제 좌표 전송 ──
            data[i * 6 + 0] = center.z;   // ROS cx
            data[i * 6 + 1] = -center.x;  // ROS cy
            data[i * 6 + 2] = center.y;   // ROS cz
            data[i * 6 + 3] = e.z;        // ROS ex
            data[i * 6 + 4] = e.x;        // ROS ey
            data[i * 6 + 5] = e.y;        // ROS ez
        }

        var msg = new Float32MultiArrayMsg { data = data };
        ros.Publish(topicName, msg);
    }

    static Bounds GetWorldBounds(MachineHeat machine)
    {
        Renderer rend = machine.GetComponentInChildren<Renderer>();
        if (rend != null)
            return rend.bounds;

        Collider col = machine.GetComponentInChildren<Collider>();
        if (col != null)
            return col.bounds;

        return new Bounds(machine.transform.position, Vector3.one);
    }
}
