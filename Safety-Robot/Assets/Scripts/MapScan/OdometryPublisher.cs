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

    // 등록 핸드셰이크가 끝나기 전에 발행하면 "Not registered" 에러가 나므로,
    // 시작 후 잠깐 기다렸다가 발행을 시작한다.
    [Tooltip("ROS 등록 완료를 기다리는 시작 지연(초)")]
    public float startupDelay = 1.5f;
    float _startupTimer = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>("/odom");
        ros.RegisterPublisher<TFMessageMsg>("/tf");
    }

    void Update()
    {
        // 등록 완료 대기 (Not registered 에러 방지)
        if (_startupTimer < startupDelay)
        {
            _startupTimer += Time.deltaTime;
            return;
        }

        timer += Time.deltaTime;
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
