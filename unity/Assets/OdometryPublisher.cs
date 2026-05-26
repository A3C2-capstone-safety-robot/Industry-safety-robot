using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

/// <summary>
/// Unity 로봇 위치/방향을 ROS odom→base_link TF로 발행
/// Player 루트 오브젝트에 붙여서 사용 (CharacterController가 있는 오브젝트)
/// </summary>
public class OdometryPublisher : MonoBehaviour
{
    [Header("Frame IDs")]
    public string odomFrame = "odom";
    public string baseFrame = "base_link";

    [Header("Publish Rate")]
    public float publishHz = 20f;

    ROSConnection ros;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>("/odom");
        ros.RegisterPublisher<TFMessageMsg>("/tf");
    }

<<<<<<< Updated upstream:unity/Assets/OdometryPublisher.cs
    void Update()
    {
        timer += Time.deltaTime;
=======
    void FixedUpdate()
    {
        timer += Time.fixedDeltaTime;
>>>>>>> Stashed changes:Safety-Robot/Assets/OdometryPublisher.cs
        if (timer < 1f / publishHz) return;
        timer = 0f;
        Publish();
    }

    void Publish()
    {
        // Unity(왼손계, Y-up) → ROS(오른손계, Z-up) 좌표 변환
        // ros.x = unity.z (forward), ros.y = -unity.x (left)
        float rx = transform.position.z;
        float ry = -transform.position.x;

        // yaw: Unity는 Y축 기준 시계방향, ROS는 Z축 기준 반시계방향
        float rosYaw = -transform.eulerAngles.y * Mathf.Deg2Rad;
        float sinY = Mathf.Sin(rosYaw * 0.5f);
        float cosY = Mathf.Cos(rosYaw * 0.5f);

        var stamp = GetRosTime();
        var pos = new PointMsg { x = rx, y = ry, z = 0 };
        var rot = new QuaternionMsg { x = 0, y = 0, z = sinY, w = cosY };

        // /odom 토픽 발행 (nav_msgs/Odometry)
        ros.Publish("/odom", new OdometryMsg
        {
            header = new HeaderMsg { frame_id = odomFrame, stamp = stamp },
            child_frame_id = baseFrame,
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg { position = pos, orientation = rot }
            }
        });

        // /tf 발행 — odom→base_link 동적 변환
        ros.Publish("/tf", new TFMessageMsg
        {
            transforms = new[]
            {
                new TransformStampedMsg
                {
                    header = new HeaderMsg { frame_id = odomFrame, stamp = stamp },
                    child_frame_id = baseFrame,
                    transform = new TransformMsg
                    {
                        translation = new Vector3Msg { x = rx, y = ry, z = 0 },
                        rotation = rot
                    }
                }
            }
        });
    }

    static RosMessageTypes.BuiltinInterfaces.TimeMsg GetRosTime()
    {
        long ms = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)(ms / 1000),
            nanosec = (uint)(ms % 1000 * 1_000_000)
        };
    }
}
