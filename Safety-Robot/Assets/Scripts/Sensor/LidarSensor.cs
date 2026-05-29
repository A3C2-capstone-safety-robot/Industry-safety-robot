using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

/// <summary>
/// 보고서 스펙: Physics.Raycast 360도, 10Hz, sensor_msgs/LaserScan 발행
/// 로봇 GameObject의 LiDAR 위치 자식 오브젝트에 붙여서 사용
/// </summary>
public class LidarSensor : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/scan";
    public string tfTopic = "/tf";
    public string baseFrameId = "base_link";
    public string scanFrameId = "base_scan";

    [Header("LiDAR 스펙")]
    public int rayCount = 360;
    public float maxRange = 10f;
    public float minRange = 0.1f;
    public float publishHz = 10f;

    [Header("Raycast")]
    [Tooltip("LiDAR 레이캐스트에서 무시할 레이어")]
    public LayerMask ignoreLayers;
    [Tooltip("센서 바로 앞 자기 몸체 오탐을 줄이기 위한 시작 오프셋")]
    public float rayOriginOffset = 0.02f;

    [Header("Frame Reference")]
    [Tooltip("base_link로 볼 기준 Transform. 비워두면 로봇 루트 사용")]
    public Transform baseFrameTransform;

    ROSConnection ros;
    float timer;
    Transform cachedBaseFrame;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);

        cachedBaseFrame = baseFrameTransform != null ? baseFrameTransform : transform.root;
        if (cachedBaseFrame == null)
            cachedBaseFrame = transform;
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishHz) return;
        timer = 0f;
        PublishScan();
    }

    void PublishScan()
    {
        float angleStep = 2f * Mathf.PI / rayCount;
        float[] ranges = new float[rayCount];
        var stamp = GetRosTime();

        for (int i = 0; i < rayCount; i++)
        {
            float angle = i * angleStep;
            // angle=0 → 로컬 +Z(정면), CCW 증가 — ROS LaserScan 규약(angle=0=+X of base_scan) 일치
            Vector3 dir = new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));
            Vector3 worldDir = transform.TransformDirection(dir).normalized;
            Vector3 origin = transform.position + worldDir * rayOriginOffset;
            ranges[i] = FindNearestValidHit(origin, worldDir);
        }

        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = scanFrameId,
                stamp = stamp
            },
            angle_min = 0f,
            angle_max = 2f * Mathf.PI,
            angle_increment = angleStep,
            time_increment = 0f,
            scan_time = 1f / publishHz,
            range_min = minRange,
            range_max = maxRange,
            ranges = ranges,
            intensities = new float[rayCount]
        };

        ros.Publish(topicName, msg);
        PublishBaseScanTf(stamp);
    }

    float FindNearestValidHit(Vector3 origin, Vector3 direction)
    {
        RaycastHit[] hits = Physics.RaycastAll(
            origin,
            direction,
            maxRange,
            ~ignoreLayers,
            QueryTriggerInteraction.Ignore);

        float nearest = float.PositiveInfinity;
        for (int i = 0; i < hits.Length; i++)
        {
            var hit = hits[i];
            if (IsSelfHit(hit.transform))
                continue;

            if (hit.distance < nearest)
                nearest = hit.distance;
        }

        return nearest;
    }

    bool IsSelfHit(Transform hitTransform)
    {
        return hitTransform == transform || hitTransform.IsChildOf(cachedBaseFrame);
    }

    void PublishBaseScanTf(RosMessageTypes.BuiltinInterfaces.TimeMsg stamp)
    {
        if (cachedBaseFrame == null)
            return;

        Vector3 localPos = cachedBaseFrame.InverseTransformPoint(transform.position);
        Quaternion localRot = Quaternion.Inverse(cachedBaseFrame.rotation) * transform.rotation;

        // Unity local offset -> ROS local offset
        var translation = new Vector3Msg
        {
            x = localPos.z,
            y = -localPos.x,
            z = localPos.y
        };

        float rosYaw = -localRot.eulerAngles.y * Mathf.Deg2Rad;
        float halfYaw = rosYaw * 0.5f;
        var rotation = new QuaternionMsg
        {
            x = 0,
            y = 0,
            z = Mathf.Sin(halfYaw),
            w = Mathf.Cos(halfYaw)
        };

        ros.Publish(tfTopic, new TFMessageMsg
        {
            transforms = new[]
            {
                new TransformStampedMsg
                {
                    header = new HeaderMsg
                    {
                        frame_id = baseFrameId,
                        stamp = stamp
                    },
                    child_frame_id = scanFrameId,
                    transform = new TransformMsg
                    {
                        translation = translation,
                        rotation = rotation
                    }
                }
            }
        });
    }

    static RosMessageTypes.BuiltinInterfaces.TimeMsg GetRosTime()
    {
        // Unity Time.time은 0 기준이라 ROS TF 룩업 실패 → 실제 UTC 시각 사용
        long ms = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)(ms / 1000),
            nanosec = (uint)(ms % 1000 * 1_000_000)
        };
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        int step = Mathf.Max(1, rayCount / 36);   // ← 수정: 0 방지
        for (int i = 0; i < rayCount; i += step)
        {
            float angle = i * (2f * Mathf.PI / rayCount);
            Vector3 dir = transform.TransformDirection(
                new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle)));
            Gizmos.DrawRay(transform.position, dir * maxRange);
        }
    }
}
