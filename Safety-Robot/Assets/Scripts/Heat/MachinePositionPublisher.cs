//// MachinePositionPublisher.cs
//// 각 설비(Machine)의 Unity 월드 좌표를 ROS /machine_world_positions 토픽으로 발행.
//// OverheatScenarioManager와 같은 오브젝트에 추가하거나 별도 오브젝트에 추가.
//using UnityEngine;
//using Unity.Robotics.ROSTCPConnector;
//using RosMessageTypes.Std;
//using System.Linq;

//public class MachinePositionPublisher : MonoBehaviour
//{
//    [Header("설비 목록 (OverheatScenarioManager와 동일하게 연결)")]
//    public MachineHeat[] machines;

//    [Header("ROS 설정")]
//    public string topicName  = "/machine_world_positions";
//    public string frameId    = "map";
//    public float  publishRate = 1f; // Hz (위치는 자주 바꾸지 않아도 됨)

//    private ROSConnection ros;

//    void Start()
//    {
//        ros = ROSConnection.GetOrCreateInstance();
//        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);
//        InvokeRepeating(nameof(PublishPositions), 1f, 1f / publishRate);
//    }

//    void PublishPositions()
//    {
//        // 포맷: [x0, y0, z0,  x1, y1, z1,  ...] (설비 순서 = machines 배열 순서)
//        // Unity 좌표계(Left-handed) → ROS 좌표계(Right-handed) 변환
//        //   ROS x =  Unity z
//        //   ROS y = -Unity x
//        //   ROS z =  Unity y
//        float[] data = new float[machines.Length * 3];
//        for (int i = 0; i < machines.Length; i++)
//        {
//            Vector3 u = machines[i].transform.position;
//            data[i * 3]     =  u.z;  // ROS x
//            data[i * 3 + 1] = -u.x;  // ROS y
//            data[i * 3 + 2] =  u.y;  // ROS z
//        }

//        var msg  = new Float32MultiArrayMsg();
//        msg.data = data;
//        ros.Publish(topicName, msg);
//    }
//}
// MachinePositionPublisher.cs  (v2 — 바운딩박스 extents 추가)
// 설비 중심 좌표(3) + 바운딩박스 반크기 extents(3) = 설비당 6개 float 발행
// 포맷: [cx0,cy0,cz0, ex0,ey0,ez0,  cx1,cy1,cz1, ex1,ey1,ez1, ...]
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class MachinePositionPublisher : MonoBehaviour
{
    [Header("설비 목록 (OverheatScenarioManager와 동일하게 연결)")]
    public MachineHeat[] machines;

    [Header("ROS 설정")]
    public string topicName = "/machine_world_positions";
    public string frameId = "map";
    public float publishRate = 1f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);
        InvokeRepeating(nameof(PublishPositions), 1f, 1f / publishRate);
    }

    void PublishPositions()
    {
        // 설비당 6 floats: [center(3), extents(3)]
        // Unity(Left-handed) → ROS(Right-handed) 축 변환
        //   ROS x =  Unity z
        //   ROS y = -Unity x
        //   ROS z =  Unity y
        // extents 는 부호 없는 반크기이므로 절댓값만 축 재배치
        float[] data = new float[machines.Length * 6];

        for (int i = 0; i < machines.Length; i++)
        {
            Bounds bounds = GetWorldBounds(machines[i]);
            Vector3 center = bounds.center;
            Vector3 e = bounds.extents;

            // ── 중심 좌표 ──────────────────────────────────────
            data[i * 6 + 0] = center.z;   // ROS cx
            data[i * 6 + 1] = -center.x;  // ROS cy
            data[i * 6 + 2] = center.y;   // ROS cz

            // 축 재배치 (extents는 절댓값 반크기 → 부호 불필요)
            data[i * 6 + 3] = e.z;    // ROS ex  ← Unity ez
            data[i * 6 + 4] = e.x;    // ROS ey  ← Unity ex
            data[i * 6 + 5] = e.y;    // ROS ez  ← Unity ey
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
