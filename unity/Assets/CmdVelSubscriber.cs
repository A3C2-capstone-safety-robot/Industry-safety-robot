using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CmdVelSubscriber : MonoBehaviour
{
    [Header("ROS 설정")]
<<<<<<< Updated upstream:unity/Assets/CmdVelSubscriber.cs
    public string topicName = "/cmd_vel";
=======
    public string topicName = "/cmd_vel_nav";
>>>>>>> Stashed changes:Safety-Robot/Assets/CmdVelSubscriber.cs

    [Header("이동 설정")]
    public float linearSpeed = 2.0f;
    public float angularSpeed = 100.0f;

    private ROSConnection ros;

    // ROS에서 받은 속도값 저장
    private float linearVelocity = 0f;
    private float angularVelocity = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // /cmd_vel subscribe
        ros.Subscribe<TwistMsg>(topicName, CmdVelCallback);
    }

    void CmdVelCallback(TwistMsg msg)
    {
        // 전진/후진 속도
        linearVelocity = (float)msg.linear.x;

        // 회전 속도
        angularVelocity = (float)msg.angular.z;
    }

    void Update()
    {
        // 전진/후진 이동
        transform.Translate(
            Vector3.forward * linearVelocity * linearSpeed * Time.deltaTime
        );

        // 좌우 회전
        transform.Rotate(
            Vector3.up,
            -angularVelocity * angularSpeed * Time.deltaTime
        );
    }
}