// CmdVelSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CmdVelSubscriber : MonoBehaviour
{
    public string topicName = "/cmd_vel";

    [Header("이동 설정")]
    public float linearSpeedScale = 1f;    // 선속도 스케일
    public float angularSpeedScale = 1f;   // 각속도 스케일

    [Header("충돌 방지")]
    [Tooltip("true이면 이 스크립트가 transform 이동을 하지 않음 (MothSearchAlgorithm 직접 이동 모드용)")]
    public bool suppressMovement = false;

    private Rigidbody rb;
    private Vector3 targetLinearVel;
    private float targetAngularVel;

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>(topicName, OnCmdVelReceived);
    }

    void OnCmdVelReceived(TwistMsg msg)
    {
        // ROS cmd_vel → Unity 이동 명령 변환
        // ROS: linear.x = 전진, angular.z = 회전
        targetLinearVel = transform.forward * (float)msg.linear.x * linearSpeedScale;
        targetAngularVel = -(float)msg.angular.z * angularSpeedScale; // 방향 반전
    }

    void FixedUpdate()
    {
        if (suppressMovement)
        {
            StopRigidbody();
            return;
        }

        if (rb != null)
        {
            rb.linearVelocity = new Vector3(targetLinearVel.x, rb.linearVelocity.y, targetLinearVel.z);
            rb.angularVelocity = new Vector3(0f, targetAngularVel, 0f);
        }
        else
        {
            // Rigidbody가 없으면 Transform 직접 이동
            transform.position += targetLinearVel * Time.fixedDeltaTime;
            transform.Rotate(0f, targetAngularVel * Mathf.Rad2Deg * Time.fixedDeltaTime, 0f);
        }
    }

    public void ResetCommand()
    {
        targetLinearVel = Vector3.zero;
        targetAngularVel = 0f;
        StopRigidbody();
    }

    void StopRigidbody()
    {
        if (rb == null)
            return;

        rb.linearVelocity = new Vector3(0f, rb.linearVelocity.y, 0f);
        rb.angularVelocity = Vector3.zero;
    }
}
