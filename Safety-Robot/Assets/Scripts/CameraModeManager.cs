using UnityEngine;

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

    public CameraViewMode CurrentMode { get; private set; }

    void Start()
    {
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

    void ApplyMode(CameraViewMode mode)
    {
        CurrentMode = mode;

        bool thirdPersonActive = mode == CameraViewMode.ThirdPerson;
        SetCameraState(thirdPersonCamera, thirdPersonActive);
        SetCameraState(firstPersonCamera, !thirdPersonActive);

        SetControllerState(thirdPersonController, thirdPersonActive);
        SetControllerState(firstPersonController, !thirdPersonActive);
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
