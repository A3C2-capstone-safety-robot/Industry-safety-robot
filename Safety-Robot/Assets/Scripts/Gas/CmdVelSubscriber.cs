// CmdVelSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

// Nav2(또는 MothSearch)가 보낸 /cmd_vel 을 받아 로봇을 움직인다.
// CharacterController.Move 로 이동 → 벽/장애물 콜라이더에 막힘 (안 뚫림).
public class CmdVelSubscriber : MonoBehaviour
{
    public string topicName = "/cmd_vel";

    [Header("이동 설정")]
    public float linearSpeedScale = 1f;    // 선속도 스케일
    public float angularSpeedScale = 1f;   // 각속도 스케일

    [Header("충돌 방지")]
    [Tooltip("true이면 이 스크립트가 이동을 하지 않음")]
    public bool suppressMovement = false;

    private CharacterController controller;
    private float linearX = 0f;    // m/s (전진/후진)
    private float angularZ = 0f;   // rad/s (회전)
    private float yVelocity = 0f;
    private const float Gravity = -9.81f;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        if (controller == null)
            controller = GetComponentInParent<CharacterController>();
        if (controller == null)
            Debug.LogWarning("[CmdVelSubscriber] CharacterController 없음 — 충돌 처리 안 됨(폴백). " +
                             "로봇에 CharacterController 를 두는 걸 권장.", this);

        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>(topicName, OnCmdVelReceived);
    }

    void OnCmdVelReceived(TwistMsg msg)
    {
        // ROS: linear.x = 전진(m/s), angular.z = 회전(rad/s, +가 좌회전)
        linearX = (float)msg.linear.x;
        angularZ = (float)msg.angular.z;
    }

    void Update()
    {
        if (suppressMovement)
            return;

        // 회전 (ROS +angular.z = 좌회전(CCW) → Unity는 시계방향이 +라 부호 반전)
        float turnDeg = -angularZ * angularSpeedScale * Mathf.Rad2Deg * Time.deltaTime;
        transform.Rotate(0f, turnDeg, 0f);

        if (controller != null)
        {
            // 중력으로 바닥에 붙어있게 유지
            if (controller.isGrounded && yVelocity < 0f)
                yVelocity = -2f;
            else
                yVelocity += Gravity * Time.deltaTime;

            // 전진/후진(수평) + 중력(수직) → CharacterController.Move 로 충돌 처리
            Vector3 horizontal = transform.forward * linearX * linearSpeedScale;
            Vector3 motion = new Vector3(horizontal.x, yVelocity, horizontal.z);
            controller.Move(motion * Time.deltaTime);
        }
        else
        {
            // CharacterController 가 없을 때만 폴백 (충돌 처리 안 됨)
            transform.position += transform.forward * linearX * linearSpeedScale * Time.deltaTime;
        }
    }

    public void ResetCommand()
    {
        linearX = 0f;
        angularZ = 0f;
    }
}
