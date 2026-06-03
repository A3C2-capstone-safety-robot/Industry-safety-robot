using UnityEngine;

/// <summary>
/// 티칭 전용 수동 운전 컴포넌트.
/// 이 컴포넌트가 '활성화(enabled)'된 동안에만 WASD 로 로봇을 운전한다.
/// CharacterController.Move 로 이동 → 벽/기계 콜라이더에 막힘 (안 뚫림).
/// 평소엔 비활성(Inspector 체크 해제)으로 두고, CameraModeManager 가 켜고 끈다.
/// </summary>
public class ManualTeachController : MonoBehaviour
{
    [Header("이동 설정")]
    public float moveSpeed = 1.5f;     // 전진/후진 속도 (m/s)
    public float rotateSpeed = 80f;    // 조향 회전 속도 (deg/s)
    public float inputDeadZone = 0.01f;
    [Tooltip("꺼두면 자동차처럼 W/S로 움직일 때만 A/D 조향이 적용됨")]
    public bool allowTurnInPlace = false;
    [Tooltip("후진 중 A/D 조향 방향을 자동차처럼 반대로 적용")]
    public bool invertSteeringWhenReversing = true;

    public bool IsCommandedMoving { get; private set; }
    public float ForwardInput { get; private set; }
    public float TurnInput { get; private set; }

    private CharacterController controller;
    private float yVelocity = 0f;
    private const float Gravity = -9.81f;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        if (controller == null)
            controller = GetComponentInParent<CharacterController>();
    }

    void Update()
    {
        ForwardInput = 0f;
        TurnInput = 0f;
        if (Input.GetKey(KeyCode.W)) ForwardInput += 1f;
        if (Input.GetKey(KeyCode.S)) ForwardInput -= 1f;
        if (Input.GetKey(KeyCode.A)) TurnInput -= 1f;
        if (Input.GetKey(KeyCode.D)) TurnInput += 1f;

        bool hasForwardInput = Mathf.Abs(ForwardInput) > inputDeadZone;
        bool hasTurnInput = Mathf.Abs(TurnInput) > inputDeadZone;
        IsCommandedMoving = hasForwardInput || (allowTurnInPlace && hasTurnInput);

        // 자동차식 조향: 전/후진 입력이 있을 때 A/D로 진행 방향을 꺾는다.
        if (hasTurnInput && (hasForwardInput || allowTurnInPlace))
        {
            float steeringDirection = 1f;
            if (invertSteeringWhenReversing && ForwardInput < -inputDeadZone)
                steeringDirection = -1f;

            float speedFactor = hasForwardInput ? Mathf.Abs(ForwardInput) : 1f;
            transform.Rotate(Vector3.up, TurnInput * steeringDirection * rotateSpeed * speedFactor * Time.deltaTime);
        }

        if (controller != null)
        {
            // 중력으로 바닥에 붙어있게 유지
            if (controller.isGrounded && yVelocity < 0f)
                yVelocity = -2f;
            else
                yVelocity += Gravity * Time.deltaTime;

            // 전진/후진(수평) + 중력(수직) → CharacterController.Move 로 충돌 처리
            Vector3 horizontal = transform.forward * ForwardInput * moveSpeed;
            Vector3 motion = new Vector3(horizontal.x, yVelocity, horizontal.z);
            controller.Move(motion * Time.deltaTime);
        }
        else
        {
            // CharacterController 가 없을 때만 폴백 (충돌 처리 안 됨)
            transform.Translate(Vector3.forward * ForwardInput * moveSpeed * Time.deltaTime, Space.Self);
        }
    }

    void OnDisable()
    {
        ForwardInput = 0f;
        TurnInput = 0f;
        IsCommandedMoving = false;
    }
}
