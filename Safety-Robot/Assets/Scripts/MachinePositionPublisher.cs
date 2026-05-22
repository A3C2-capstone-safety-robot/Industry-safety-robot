// MachinePositionPublisher.cs
// 각 설비(Machine)의 Unity 월드 좌표를 ROS /machine_world_positions 토픽으로 발행.
// OverheatScenarioManager와 같은 오브젝트에 추가하거나 별도 오브젝트에 추가.
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Linq;

public class MachinePositionPublisher : MonoBehaviour
{
    [Header("설비 목록 (OverheatScenarioManager와 동일하게 연결)")]
    public MachineHeat[] machines;

    [Header("ROS 설정")]
    public string topicName  = "/machine_world_positions";
    public string frameId    = "map";
    public float  publishRate = 1f; // Hz (위치는 자주 바꾸지 않아도 됨)

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);
        InvokeRepeating(nameof(PublishPositions), 1f, 1f / publishRate);
    }

    void PublishPositions()
    {
        // 포맷: [x0, y0, z0,  x1, y1, z1,  ...] (설비 순서 = machines 배열 순서)
        // Unity 좌표계(Left-handed) → ROS 좌표계(Right-handed) 변환
        //   ROS x =  Unity z
        //   ROS y = -Unity x
        //   ROS z =  Unity y
        float[] data = new float[machines.Length * 3];
        for (int i = 0; i < machines.Length; i++)
        {
            Vector3 u = machines[i].transform.position;
            data[i * 3]     =  u.z;  // ROS x
            data[i * 3 + 1] = -u.x;  // ROS y
            data[i * 3 + 2] =  u.y;  // ROS z
        }

        var msg  = new Float32MultiArrayMsg();
        msg.data = data;
        ros.Publish(topicName, msg);
    }
}
