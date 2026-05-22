// LiDARSensor.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class LiDARSensor : MonoBehaviour
{
    [Header("LiDAR 스펙")]
    public int numRays = 360;          // 레이 개수 (해상도)
    public float maxRange = 20f;       // 최대 측정 거리 (m)
    public float minRange = 0.1f;      // 최소 측정 거리 (m)
    public float scanFrequency = 10f;  // Hz
    public float verticalOffset = 0.3f; // 로봇 바닥에서 LiDAR까지 높이

    [Header("ROS 설정")]
    public string topicName = "/scan";
    public string frameId = "lidar_link";

    // 시각화 옵션 (Scene 뷰에서 레이 확인용)
    public bool showDebugRays = true;
    public Color hitColor = Color.red;
    public Color missColor = Color.green;

    private ROSConnection ros;
    private float timer = 0f;
    private bool isReady = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        
        Invoke(nameof(EnablePublishing), 3.0f);
    }

    void EnablePublishing()
    {
        isReady = true;
    }

    void FixedUpdate()
    {
        if (!isReady) return;
        
        timer += Time.fixedDeltaTime;
        if (timer < 1f / scanFrequency) return;
        timer = 0f;

        PublishScan();
    }

    void PublishScan()
    {
        if (!isReady) return;
        
        float angleIncrement = 2f * Mathf.PI / numRays; // 레이 사이 각도 간격
        float[] ranges = new float[numRays];
        float[] intensities = new float[numRays];

        // 레이 시작 위치 (LiDAR 높이 보정)
        Vector3 origin = transform.position + Vector3.up * verticalOffset;

        for (int i = 0; i < numRays; i++)
        {
            // i번째 레이의 각도 계산
            float angle = i * angleIncrement;

            // Unity 좌표계에서의 방향 벡터
            // Unity는 Y-up 좌표계이므로 XZ 평면에서 회전
            Vector3 direction = new Vector3(
                Mathf.Cos(angle + transform.eulerAngles.y * Mathf.Deg2Rad),
                0f,
                Mathf.Sin(angle + transform.eulerAngles.y * Mathf.Deg2Rad)
            );

            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, maxRange))
            {
                float distance = hit.distance;
                ranges[i] = (distance >= minRange) ? distance : float.PositiveInfinity;
                intensities[i] = 1.0f; // 뭔가 맞았음

                if (showDebugRays)
                    Debug.DrawRay(origin, direction * distance, hitColor, 1f / scanFrequency);
            }
            else
            {
                ranges[i] = float.PositiveInfinity; // 범위 내 장애물 없음
                intensities[i] = 0f;

                if (showDebugRays)
                    Debug.DrawRay(origin, direction * maxRange, missColor, 1f / scanFrequency);
            }
        }

        // LaserScan 메시지 구성
        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                //stamp = RosUtil.GetCurrentTimeMsg()
            },
            angle_min = 0f,
            angle_max = 2f * Mathf.PI,
            angle_increment = angleIncrement,
            time_increment = 0f,
            scan_time = 1f / scanFrequency,
            range_min = minRange,
            range_max = maxRange,
            ranges = ranges,
            intensities = intensities
        };

        ros.Publish(topicName, msg);
    }
}