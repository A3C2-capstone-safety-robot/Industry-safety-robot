using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// 보고서 스펙: Physics.Raycast 360도, 10Hz, sensor_msgs/LaserScan 발행
/// 로봇 GameObject의 LiDAR 위치 자식 오브젝트에 붙여서 사용
/// </summary>
public class LidarSensor : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/scan";

    [Header("LiDAR 스펙")]
    public int rayCount = 360;
    public float maxRange = 10f;
    public float minRange = 0.1f;
    public float publishHz = 10f;

    ROSConnection ros;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
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

        for (int i = 0; i < rayCount; i++)
        {
            float angle = i * angleStep;
            // angle=0 → 로컬 +Z(정면), CCW 증가 — ROS LaserScan 규약(angle=0=+X of base_scan) 일치
            Vector3 dir = new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));

            ranges[i] = Physics.Raycast(
                transform.position,
                transform.TransformDirection(dir),
                out RaycastHit hit,
                maxRange)
                ? hit.distance
                : float.PositiveInfinity;
        }

        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_scan",
                stamp = GetRosTime()
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
    }

    static RosMessageTypes.BuiltinInterfaces.TimeMsg GetRosTime()
    {
        // Unity Time.time은 0 기준이라 ROS TF 룩업 실패 → 실제 UTC 시각 사용
        long ms = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (uint)(ms / 1000),
            nanosec = (uint)(ms % 1000 * 1_000_000)
        };
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        int step = rayCount / 36;
        for (int i = 0; i < rayCount; i += step)
        {
            float angle = i * (2f * Mathf.PI / rayCount);
            Vector3 dir = transform.TransformDirection(
                new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle)));
            Gizmos.DrawRay(transform.position, dir * maxRange);
        }
    }
}
