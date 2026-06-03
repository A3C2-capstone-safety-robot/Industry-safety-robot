using UnityEngine;

/// <summary>
/// 3인칭 팔로우 카메라.
/// target(로봇)을 일정 거리에서 따라다니며, 마우스로 주위를 둘러볼 수 있다(궤도 회전).
/// 수동조작(티칭) 중에만 켜지도록 CameraModeManager 가 enable/disable 한다.
/// 3인칭 카메라(DebugFreeCamera) 오브젝트에 붙여서 사용.
/// </summary>
public class ThirdPersonFollowCamera : MonoBehaviour
{
    [Header("따라갈 대상")]
    public Transform target;            // 로봇

    [Header("거리/높이")]
    public float distance = 5f;         // 로봇 뒤로 떨어진 거리
    public float height = 2.5f;         // 바라보는 지점 높이

    [Header("마우스 회전")]
    public float mouseSensitivity = 3f;
    public float minPitch = -10f;       // 아래로 내려다보는 한계
    public float maxPitch = 70f;        // 위로 올려다보는 한계
    [Tooltip("체크하면 우클릭 누르는 동안에만 회전. 해제하면 항상 마우스로 회전")]
    public bool rotateOnlyWhileRightMouseHeld = false;

    private float yaw = 0f;
    private float pitch = 20f;

    void OnEnable()
    {
        // 켜질 때 현재 각도로 초기화
        Vector3 e = transform.eulerAngles;
        yaw = e.y;
        pitch = Mathf.Clamp(e.x, minPitch, maxPitch);
    }

    void LateUpdate()
    {
        if (target == null) return;

        bool canRotate = !rotateOnlyWhileRightMouseHeld || Input.GetMouseButton(1);
        if (canRotate)
        {
            yaw += Input.GetAxis("Mouse X") * mouseSensitivity;
            pitch -= Input.GetAxis("Mouse Y") * mouseSensitivity;
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        }

        Quaternion rot = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 focus = target.position + Vector3.up * height;

        // 로봇 뒤쪽 거리만큼 떨어진 위치로 카메라 배치
        transform.position = focus - (rot * Vector3.forward) * distance;
        transform.rotation = Quaternion.LookRotation(focus - transform.position, Vector3.up);
    }
}
