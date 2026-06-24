using UnityEngine;
using UnityEngine.UI;

public class CameraModeManager : MonoBehaviour
{
    public enum CameraViewMode
    {
        ThirdPerson,
        FirstPerson
    }

    [Header("카메라")]
    public Camera thirdPersonCamera;
    public Camera firstPersonCamera;

    [Header("선택: 카메라별 제어 스크립트")]
    public MonoBehaviour thirdPersonController;
    public MonoBehaviour firstPersonController;

    [Header("시작 모드")]
    public CameraViewMode defaultMode = CameraViewMode.ThirdPerson;

    [Header("선택: 수동 조작(티칭) 컨트롤러")]
    [Tooltip("로봇 오브젝트를 드래그하면 ManualTeachController 가 자동으로 잡힘. 수동조작 ON 동안에만 활성화됨")]
    public ManualTeachController manualDriveController;
    [Tooltip("수동조작 ON 동안 /cmd_vel 이동이 WASD와 충돌하지 않도록 잠시 막음")]
    public CmdVelSubscriber cmdVelSubscriber;

    [Header("선택: 수동조작 버튼 색상")]
    [Tooltip("버튼의 Image 를 연결하면 ON/OFF 에 따라 색이 바뀜")]
    public Image manualButtonImage;
    public Color manualOnColor = new Color(0.30f, 0.78f, 0.40f);   // ON: 초록
    public Color manualOffColor = new Color(0.85f, 0.85f, 0.85f);  // OFF: 회색
    [Tooltip("버튼 글자(선택). 연결하면 ON/OFF 텍스트로 바뀜")]
    public Text manualButtonLabel;

    [Header("선택: 수동조작 중 3인칭 팔로우 카메라")]
    [Tooltip("3인칭 카메라의 ThirdPersonFollowCamera 를 연결. 수동조작 ON 동안 로봇을 따라다니며 마우스로 둘러봄")]
    public ThirdPersonFollowCamera followCamController;

    [Header("선택: 팔로우 모드 버튼 색상")]
    [Tooltip("버튼의 Image 를 연결하면 ON/OFF 에 따라 색이 바뀜")]
    public Image followButtonImage;
    public Color followOnColor = new Color(0.30f, 0.60f, 0.95f);   // ON: 파랑
    public Color followOffColor = new Color(0.85f, 0.85f, 0.85f);  // OFF: 회색
    [Tooltip("버튼 글자(선택). 연결하면 ON/OFF 텍스트로 바뀜")]
    public Text followButtonLabel;

    public CameraViewMode CurrentMode { get; private set; }
    public bool ManualDriveActive { get; private set; }
    public bool FollowModeActive { get; private set; }
    private bool previousCmdVelSuppressMovement;

    void Start()
    {
        if (cmdVelSubscriber == null && manualDriveController != null)
            cmdVelSubscriber = manualDriveController.GetComponent<CmdVelSubscriber>();

        // 시작 시 수동조작은 꺼둠 (평소엔 WASD 로 로봇 안 움직임)
        ManualDriveActive = false;
        if (manualDriveController != null)
            manualDriveController.enabled = false;
        if (followCamController != null)
            followCamController.enabled = false;

        UpdateManualButtonVisual();
        UpdateFollowButtonVisual();
        ApplyMode(defaultMode);
    }

    // ── 자율주행 팔로우 모드 토글 — UI 버튼의 OnClick 에 연결 ──
    // ON: 3인칭 카메라가 로봇을 따라다님 (자율주행 구경용, 마우스로 둘러보기 가능)
    // OFF: 카메라가 로봇과 분리되어 다시 자유 시점
    public void ToggleFollowMode()
    {
        SetFollowMode(!FollowModeActive);
    }

    public void SetFollowMode(bool on)
    {
        FollowModeActive = on;

        // 팔로우는 3인칭에서만 의미 있음 — 켜면 3인칭으로 강제 전환
        ApplyMode(on ? CameraViewMode.ThirdPerson : CurrentMode);

        UpdateFollowButtonVisual();
        Debug.Log(on
            ? "[CameraModeManager] 팔로우 모드 ON — 카메라가 로봇 추적"
            : "[CameraModeManager] 팔로우 모드 OFF — 자유 시점 복원");
    }

    void UpdateFollowButtonVisual()
    {
        if (followButtonImage != null)
            followButtonImage.color = FollowModeActive ? followOnColor : followOffColor;
        if (followButtonLabel != null)
            followButtonLabel.text = FollowModeActive ? "팔로우 ON" : "팔로우 OFF";
    }

    public void SetThirdPersonView()
    {
        ApplyMode(CameraViewMode.ThirdPerson);
    }

    public void SetFirstPersonView()
    {
        ApplyMode(CameraViewMode.FirstPerson);
    }

    public void ToggleView()
    {
        ApplyMode(CurrentMode == CameraViewMode.ThirdPerson
            ? CameraViewMode.FirstPerson
            : CameraViewMode.ThirdPerson);
    }

    // ── 수동 조작(티칭) 토글 — UI 버튼의 OnClick 에 연결 ──
    public void ToggleManualDrive()
    {
        SetManualDrive(!ManualDriveActive);
    }

    public void SetManualDrive(bool on)
    {
        ManualDriveActive = on;

        if (manualDriveController != null)
            manualDriveController.enabled = on;
        SetCmdVelSuppressedForManualDrive(on);

        // 수동 조작은 3인칭 팔로우 시점 강제, 해제 시 현재 모드 복원
        // (팔로우 카메라/컨트롤러 on-off는 ApplyMode가 일괄 처리 —
        //  팔로우 모드가 켜져 있으면 수동조작을 꺼도 팔로우는 유지됨)
        ApplyMode(on ? CameraViewMode.ThirdPerson : CurrentMode);

        UpdateManualButtonVisual();

        Debug.Log(on
            ? "[CameraModeManager] 수동 조작 ON — WASD 로 로봇 운전"
            : "[CameraModeManager] 수동 조작 OFF");
    }

    void SetCmdVelSuppressedForManualDrive(bool on)
    {
        if (cmdVelSubscriber == null)
            return;

        if (on)
        {
            previousCmdVelSuppressMovement = cmdVelSubscriber.suppressMovement;
            cmdVelSubscriber.suppressMovement = true;
            cmdVelSubscriber.ResetCommand();
        }
        else
        {
            cmdVelSubscriber.ResetCommand();
            cmdVelSubscriber.suppressMovement = previousCmdVelSuppressMovement;
        }
    }

    void UpdateManualButtonVisual()
    {
        if (manualButtonImage != null)
            manualButtonImage.color = ManualDriveActive ? manualOnColor : manualOffColor;
        if (manualButtonLabel != null)
            manualButtonLabel.text = ManualDriveActive ? "수동 조작 ON" : "수동 조작 OFF";
    }

    void ApplyMode(CameraViewMode mode)
    {
        CurrentMode = mode;

        bool thirdPersonActive = mode == CameraViewMode.ThirdPerson;
        SetCameraState(thirdPersonCamera, thirdPersonActive);
        SetCameraState(firstPersonCamera, !thirdPersonActive);

        // 팔로우 카메라: (수동조작 또는 팔로우 모드) + 3인칭일 때만 ON
        bool followNow = thirdPersonActive && (ManualDriveActive || FollowModeActive);
        if (followCamController != null)
            followCamController.enabled = followNow;

        if (ManualDriveActive || followNow)
        {
            // 팔로우/수동조작 중에는 자유 카메라 컨트롤러를 꺼서 마우스 충돌 방지
            SetControllerState(thirdPersonController, false);
            SetControllerState(firstPersonController, !thirdPersonActive && !ManualDriveActive);
        }
        else
        {
            SetControllerState(thirdPersonController, thirdPersonActive);
            SetControllerState(firstPersonController, !thirdPersonActive);
        }
    }

    static void SetCameraState(Camera cam, bool isActive)
    {
        if (cam == null)
            return;

        cam.enabled = isActive;

        AudioListener listener = cam.GetComponent<AudioListener>();
        if (listener != null)
            listener.enabled = isActive;
    }

    static void SetControllerState(MonoBehaviour controller, bool isActive)
    {
        if (controller != null)
            controller.enabled = isActive;
    }
}
