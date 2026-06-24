using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// 씬 시작 시 ROS 연결 설정. 별도 빈 GameObject에 붙여서 사용.
/// ROS_IP: WSL2의 IP (wsl hostname -I 로 확인)
/// </summary>
public class ROSConnectionSetup : MonoBehaviour
{
    [Header("WSL2 IP 입력 (wsl hostname -I 로 확인)")]
    public string rosIP = "172.26.198.146";
    public int rosPort = 10000;

    void Awake()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIP;
        ros.RosPort = rosPort;
    }
}
