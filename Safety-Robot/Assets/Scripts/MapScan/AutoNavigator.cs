using UnityEngine;

/// <summary>
/// 로봇청소기식 자율 이동 — SLAM 맵 커버리지용
/// Player 루트(CharacterController가 있는 오브젝트)에 붙여서 사용
/// Tab 키로 자동/수동 전환
/// </summary>
[RequireComponent(typeof(CharacterController))]
public class AutoNavigator : MonoBehaviour
{
    [Header("이동 설정")]
    public float moveSpeed = 2f;
    public float rotateSpeed = 90f;       // 초당 회전 각도

    [Header("장애물 감지")]
    public float frontCheckDistance = 1.0f;  // 전방 레이캐스트 거리(m)

    [Header("회전 범위 (도)")]
    public float minRotateAngle = 90f;
    public float maxRotateAngle = 270f;

    CharacterController controller;
    MonoBehaviour cameraMove;   // CameraMove(UnityFactorySceneHDRP 네임스페이스) 비활성화용

    bool autoMode = false;
    bool rotating = false;
    float rotateRemain = 0f;
    float rotateDir = 1f;

    void Start()
    {
        controller = GetComponent<CharacterController>();

        // 네임스페이스가 달라도 타입 이름으로 검색
        foreach (var mb in GetComponents<MonoBehaviour>())
        {
            if (mb.GetType().Name == "CameraMove")
            {
                cameraMove = mb;
                break;
            }
        }
    }

    void Update()
    {
        // Tab 키로 자동/수동 전환
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            autoMode = !autoMode;
            if (cameraMove != null) cameraMove.enabled = !autoMode;
            Debug.Log(autoMode
                ? "[AutoNav] 자동 모드 ON — Tab 키로 수동 전환"
                : "[AutoNav] 수동 모드 ON");
        }

        if (!autoMode) return;

        // 회전 중이면 회전 완료까지 이동하지 않음
        if (rotating)
        {
            float step = rotateSpeed * Time.deltaTime;
            if (step >= rotateRemain)
            {
                transform.Rotate(0f, rotateDir * rotateRemain, 0f);
                rotating = false;
            }
            else
            {
                transform.Rotate(0f, rotateDir * step, 0f);
                rotateRemain -= step;
            }
            return;
        }

        // 전방 레이캐스트로 장애물 선감지 (0.5m 높이에서 쏨)
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        if (Physics.Raycast(origin, transform.forward, frontCheckDistance))
        {
            TriggerRotate();
        }
        else
        {
            controller.SimpleMove(transform.forward * moveSpeed);
        }
    }

    // CharacterController 충돌 시 후반응 회전 (레이캐스트 미감지 시 보완)
    void OnControllerColliderHit(ControllerColliderHit hit)
    {
        if (!autoMode || rotating) return;
        if (hit.normal.y > 0.5f) return;   // 바닥 충돌 무시
        TriggerRotate();
    }

    void TriggerRotate()
    {
        rotating = true;
        rotateRemain = Random.Range(minRotateAngle, maxRotateAngle);
        rotateDir = Random.value > 0.5f ? 1f : -1f;
    }

    // Scene 뷰에서 감지 거리 시각화
    void OnDrawGizmosSelected()
    {
        Gizmos.color = autoMode ? Color.red : Color.yellow;
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Gizmos.DrawRay(origin, transform.forward * frontCheckDistance);
    }
}
