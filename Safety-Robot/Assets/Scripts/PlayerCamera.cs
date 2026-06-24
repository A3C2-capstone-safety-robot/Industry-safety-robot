using UnityEngine;

// 자유 시점 카메라 (충돌 처리 버전)
// transform.position += ... (벽 뚫림) → CharacterController.Move() (벽/천장/바닥에 막힘)
// CharacterController 가 없으면 자동으로 추가됨.
[RequireComponent(typeof(CharacterController))]
public class PlayerCamera : MonoBehaviour
{
    [Header("이동")]
    public float moveSpeed = 5f;
    public float fastMoveMultiplier = 2.5f;

    [Header("시점")]
    public float lookSpeed = 2f;
    public bool rotateOnlyWhileRightMouseHeld = true;

    private float rotX = 0f;
    private float rotY = 0f;
    private CharacterController controller;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        rotX = transform.eulerAngles.y;
        rotY = transform.eulerAngles.x;
    }

    void Update()
    {
        // ── 시점 회전 (마우스) ──
        bool canRotate = !rotateOnlyWhileRightMouseHeld || Input.GetMouseButton(1);
        if (canRotate)
        {
            rotX += Input.GetAxis("Mouse X") * lookSpeed;
            rotY -= Input.GetAxis("Mouse Y") * lookSpeed;
            rotY = Mathf.Clamp(rotY, -80f, 80f);
            transform.rotation = Quaternion.Euler(rotY, rotX, 0f);
        }

        // ── 입력 ──
        float h = Input.GetAxis("Horizontal");
        float v = Input.GetAxis("Vertical");
        float up = 0f;
        if (Input.GetKey(KeyCode.E)) up = 1f;
        if (Input.GetKey(KeyCode.Q)) up = -1f;

        float speed = moveSpeed;
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            speed *= fastMoveMultiplier;

        // ── WASD 는 '수평' 기준으로 평탄화 ──
        // (위/아래를 봐도 천장·바닥으로 날아가지 않도록 forward 의 y 를 제거)
        Vector3 flatForward = transform.forward;
        flatForward.y = 0f;
        flatForward.Normalize();

        Vector3 flatRight = transform.right;
        flatRight.y = 0f;
        flatRight.Normalize();

        // 수평 이동(WASD) + 상하 이동(Q/E)
        Vector3 move = flatForward * v + flatRight * h + Vector3.up * up;

        // ── CharacterController.Move 로 이동 → 벽/천장/바닥 Collider 에 막힘 ──
        // (SimpleMove 는 중력 적용 + Y 무시라 자유 카메라엔 부적합. Move 사용)
        controller.Move(move * speed * Time.deltaTime);
    }
}
