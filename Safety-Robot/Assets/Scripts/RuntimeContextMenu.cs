// RuntimeContextMenu.cs
// 게임 씬에서 오브젝트 우클릭 시 가스 누출 / 과열 트리거 메뉴를 표시합니다.
// 사용법: 빈 GameObject에 이 스크립트를 부착하고 Target Camera를 연결하세요.
using UnityEngine;

public class RuntimeContextMenu : MonoBehaviour
{
    [Header("카메라 (비워두면 Camera.main 자동 사용)")]
    public Camera targetCamera;

    [Header("레이캐스트 최대 거리")]
    public float raycastDistance = 100f;

    [Header("가스 누출원 스크린 픽 반경 (px) — 콜라이더 없는 파티클 오브젝트용")]
    public float gasPickRadius = 60f;

    private enum MenuType { None, GasLeak, Heat }

    private MenuType  activeMenu    = MenuType.None;
    private GaussianPlumeModel selectedGasLeak;
    private MachineHeat        selectedMachine;
    private Rect menuRect;

    private const float MenuWidth  = 220f;
    private const float ItemHeight = 28f;
    private const float Padding    = 8f;

    // 현재 활성 카메라 — 1인칭/3인칭 전환 시 꺼진 카메라로 레이를 쏘는 버그 방지.
    // (Start에서 한 번만 캐싱하면 카메라 전환 후 우클릭이 안 먹음)
    Camera ActiveCamera()
    {
        if (targetCamera != null && targetCamera.isActiveAndEnabled)
            return targetCamera;
        if (Camera.main != null)
            return Camera.main;
        // MainCamera 태그가 없어도 활성 카메라 탐색
        foreach (var c in Camera.allCameras)
            if (c.isActiveAndEnabled)
                return c;
        return null;
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(1))
        {
            TryOpenMenu();
        }
        else if (Input.GetMouseButtonDown(0))
        {
            // 메뉴 영역 바깥 좌클릭 시 닫기
            Vector2 guiMouse = new Vector2(Input.mousePosition.x, Screen.height - Input.mousePosition.y);
            if (activeMenu != MenuType.None && !menuRect.Contains(guiMouse))
                CloseMenu();
        }
    }

    void TryOpenMenu()
    {
        Camera cam = ActiveCamera();
        if (cam == null)
        {
            Debug.LogWarning("[RuntimeContextMenu] 활성 카메라를 찾을 수 없음 — 우클릭 무시");
            return;
        }

        GaussianPlumeModel gasLeak = null;
        MachineHeat machine = null;

        // 1차: 콜라이더 기반 레이캐스트 (MachineHeat 등 메시 오브젝트용)
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit, raycastDistance))
        {
            gasLeak = hit.collider.GetComponentInParent<GaussianPlumeModel>();
            machine = hit.collider.GetComponentInParent<MachineHeat>();
        }

        // 2차: 가스 누출원은 파티클 시스템(콜라이더 없음) → 스크린 좌표 근접 탐색
        if (gasLeak == null && machine == null)
            gasLeak = FindNearestGasLeakOnScreen(cam);

        float mouseGUIy = Screen.height - Input.mousePosition.y;
        float x = Mathf.Clamp(Input.mousePosition.x, 0, Screen.width  - MenuWidth - 5);

        if (gasLeak != null)
        {
            selectedGasLeak = gasLeak;
            selectedMachine = null;
            activeMenu      = MenuType.GasLeak;
            float height    = CalcGasMenuHeight();
            float y         = Mathf.Clamp(mouseGUIy, 0, Screen.height - height - 5);
            menuRect        = new Rect(x, y, MenuWidth, height);
        }
        else if (machine != null)
        {
            selectedMachine = machine;
            selectedGasLeak = null;
            activeMenu      = MenuType.Heat;
            float height    = CalcHeatMenuHeight();
            float y         = Mathf.Clamp(mouseGUIy, 0, Screen.height - height - 5);
            menuRect        = new Rect(x, y, MenuWidth, height);
        }
        else
        {
            CloseMenu();
        }
    }

    // 가스 누출원(GaussianPlumeModel)은 파티클 오브젝트라 콜라이더가 없음.
    // leakSource 위치를 스크린에 투영해 마우스와 가장 가까운 것을 반환.
    GaussianPlumeModel FindNearestGasLeakOnScreen(Camera cam)
    {
        var plumes = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        float minDist = gasPickRadius;
        GaussianPlumeModel nearest = null;
        Vector2 mousePos = new Vector2(Input.mousePosition.x, Input.mousePosition.y);

        foreach (var p in plumes)
        {
            Transform pivot = p.leakSource != null ? p.leakSource : p.transform;
            Vector3 screenPos = cam.WorldToScreenPoint(pivot.position);
            if (screenPos.z < 0f) continue; // 카메라 뒤쪽

            float dist = Vector2.Distance(new Vector2(screenPos.x, screenPos.y), mousePos);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = p;
            }
        }
        return nearest;
    }

    void CloseMenu()
    {
        activeMenu      = MenuType.None;
        selectedGasLeak = null;
        selectedMachine = null;
    }

    void OnGUI()
    {
        if (activeMenu == MenuType.None) return;

        GUI.Box(menuRect, GUIContent.none);
        GUILayout.BeginArea(new Rect(
            menuRect.x + Padding,
            menuRect.y + Padding,
            menuRect.width  - Padding * 2,
            menuRect.height - Padding * 2));

        switch (activeMenu)
        {
            case MenuType.GasLeak when selectedGasLeak != null:
                DrawGasLeakMenu();
                break;
            case MenuType.Heat when selectedMachine != null:
                DrawHeatMenu();
                break;
            default:
                CloseMenu();
                break;
        }

        GUILayout.EndArea();
    }

    // ──────────────────────────────────────────────
    //  가스 누출 메뉴
    // ──────────────────────────────────────────────
    void DrawGasLeakMenu()
    {
        GUIStyle title = TitleStyle();
        GUILayout.Label($"[{selectedGasLeak.gameObject.name}]", title);
        GUILayout.Label($"가스 타입: {selectedGasLeak.gasType}");
        GUILayout.Label($"위험 임계: {selectedGasLeak.dangerThreshold:F0} ppm");
        GUILayout.Space(4);

        if (selectedGasLeak.isLeaking)
        {
            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("가스 누출 중지", GUILayout.Height(ItemHeight)))
            {
                selectedGasLeak.StopLeak();
                CloseMenu();
            }
        }
        else
        {
            GUI.backgroundColor = new Color(0.5f, 1f, 0.5f);
            if (GUILayout.Button("가스 누출 시작", GUILayout.Height(ItemHeight)))
            {
                selectedGasLeak.StartLeak();
                CloseMenu();
            }
        }

        GUI.backgroundColor = Color.white;
        if (GUILayout.Button("닫기", GUILayout.Height(ItemHeight)))
            CloseMenu();
    }

    // ──────────────────────────────────────────────
    //  과열 메뉴
    // ──────────────────────────────────────────────
    void DrawHeatMenu()
    {
        GUIStyle title = TitleStyle();
        GUILayout.Label($"[{selectedMachine.machineId}]", title);
        GUILayout.Label($"현재 온도: {selectedMachine.currentTemp:F1}°C  " +
                        $"({selectedMachine.Status})");
        GUILayout.Space(4);

        bool overheating = selectedMachine.Status != MachineHeat.HeatStatus.Normal;

        if (overheating)
        {
            GUI.backgroundColor = new Color(0.5f, 0.8f, 1f);
            if (GUILayout.Button("과열 중지 (냉각)", GUILayout.Height(ItemHeight)))
            {
                selectedMachine.StopOverheat();
                CloseMenu();
            }
        }
        else
        {
            GUI.backgroundColor = new Color(1f, 0.75f, 0.3f);
            if (GUILayout.Button("과열 트리거", GUILayout.Height(ItemHeight)))
            {
                selectedMachine.StartOverheat(2f);
                CloseMenu();
            }
        }

        GUI.backgroundColor = Color.white;
        if (GUILayout.Button("닫기", GUILayout.Height(ItemHeight)))
            CloseMenu();
    }

    // ──────────────────────────────────────────────
    //  헬퍼
    // ──────────────────────────────────────────────
    GUIStyle TitleStyle() => new GUIStyle(GUI.skin.label)
    {
        fontStyle  = FontStyle.Bold,
        alignment  = TextAnchor.MiddleCenter,
        fontSize   = 13
    };

    float CalcGasMenuHeight()  => Padding * 2 + 20 + 18 + 18 + 4 + ItemHeight * 2 + 4;
    float CalcHeatMenuHeight() => Padding * 2 + 20 + 18 + 4 + ItemHeight * 2 + 4;
}
