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

    public CameraViewMode CurrentMode { get; private set; }
    public bool ManualDriveActive { get; private set; }
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
        ApplyMode(defaultMode);
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

        if (on)
        {
            // 수동 조작: 3인칭 카메라로 전환 + 팔로우 카메라 ON (로봇 따라다니며 마우스 둘러보기)
            CurrentMode = CameraViewMode.ThirdPerson;
            SetCameraState(thirdPersonCamera, true);
            SetCameraState(firstPersonCamera, false);

            // 자유시점/1인칭 조작은 끔 (WASD·마우스 충돌 방지)
            SetControllerState(thirdPersonController, false);
            SetControllerState(firstPersonController, false);

            if (followCamController != null)
                followCamController.enabled = true;
        }
        else
        {
            // 팔로우 카메라 끄고, 카메라 조작을 현재 모드에 맞게 복원
            if (followCamController != null)
                followCamController.enabled = false;
            ApplyMode(CurrentMode);
        }

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

        if (ManualDriveActive)
        {
            // 수동 조작 중이면 카메라 컨트롤러는 계속 꺼둠
            SetControllerState(thirdPersonController, false);
            SetControllerState(firstPersonController, false);
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
