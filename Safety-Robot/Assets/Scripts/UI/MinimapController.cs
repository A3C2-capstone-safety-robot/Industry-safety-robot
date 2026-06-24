using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

/// <summary>
/// OccupancyGrid(/map) 기반 2D 미니맵.
/// - 회색: 미탐색 / 흰색: 이동 가능 / 검정: 벽·장애물
/// - 시안 선: 계획된 이동 경로 (waypoints) — FrontierExplorer 활성 시에만
/// - 노랑 점: 현재 목표 웨이포인트
/// - 초록 점+선: 로봇 위치·방향
/// - 색상 점: 기계 온도 상태 (파랑→노랑→주황→빨강)
/// - 색상 오버레이: 활성 가스 누출 농도 (노랑→빨강)
///
/// ★ 맵 데이터(/map)를 직접 구독하므로 FrontierExplorer와 독립적으로 동작한다.
///   순찰(Nav2) 모드처럼 Frontier 탐색이 꺼져 있어도 미니맵은 정상 표시된다.
///
/// 사용법:
/// 1. Canvas 하위에 RawImage UI 생성 후 이 컴포넌트를 붙인다.
/// 2. robotTransform 필드에 로봇 루트 Transform을 연결한다. (필수)
/// 3. explorer 필드는 선택 — 연결하면 Frontier 탐색 경로도 미니맵에 표시된다.
/// </summary>
[RequireComponent(typeof(RawImage))]
public class MinimapController : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("점유 격자 맵 토픽 (slam_toolbox 또는 map_server가 발행)")]
    public string mapTopic = "/map";

    [Header("References")]
    [Tooltip("로봇 루트 Transform (필수) — 미니맵에 로봇 위치·방향 표시용")]
    public Transform robotTransform;
    [Tooltip("선택: FrontierExplorer. 연결 시 탐색 경로(웨이포인트)도 그림. 비워두거나 꺼져 있어도 미니맵은 동작")]
    public FrontierExplorer explorer;

    [Header("Settings")]
    public int textureSize = 256;
    [Range(0.05f, 1f)]
    public float updateInterval = 0.2f;

    // ── 내부 상태 ─────────────────────────────────────────────────
    RawImage             rawImage;
    Texture2D            mapTexture;
    Color32[]            basePixels;   // 점유 격자 레이어 (맵 변경 시만 재빌드)
    Color32[]            framePixels;  // 매 업데이트마다 덮어쓰는 최종 버퍼
    float                timer;
    ExplorationMapSnapshot lastSnapshot;
    ExplorationMapSnapshot mapSnapshot; // /map 직접 구독으로 채워짐

    MachineHeat[]        machines;
    GaussianPlumeModel[] gasModels;

    // ── 색상 상수 ─────────────────────────────────────────────────
    static readonly Color32 UnknownColor  = new Color32(64,  64,  64,  255);
    static readonly Color32 FreeColor     = new Color32(220, 220, 220, 255);
    static readonly Color32 OccupiedColor = new Color32(30,  30,  30,  255);
    static readonly Color32 PathColor     = new Color32(0,   200, 255, 255);
    static readonly Color32 WpColor       = new Color32(255, 220, 0,   255);
    static readonly Color32 RobotColor    = new Color32(0,   255, 80,  255);

    // ── Unity 생명주기 ────────────────────────────────────────────
    void Awake()
    {
        rawImage    = GetComponent<RawImage>();
        mapTexture  = new Texture2D(textureSize, textureSize, TextureFormat.RGBA32, false);
        mapTexture.filterMode = FilterMode.Point;
        rawImage.texture      = mapTexture;

        int len     = textureSize * textureSize;
        basePixels  = new Color32[len];
        framePixels = new Color32[len];
    }

    void Start()
    {
        machines  = FindObjectsByType<MachineHeat>(FindObjectsSortMode.None);
        gasModels = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);

        // ★ /map 직접 구독 — FrontierExplorer 유무와 무관하게 맵 수신
        ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>(mapTopic, OnMap);

        // robotTransform 미연결 시 explorer에서 보조로 가져옴 (하위호환)
        if (robotTransform == null && explorer != null)
            robotTransform = explorer.transform;
    }

    void OnMap(OccupancyGridMsg msg)
    {
        mapSnapshot = ExplorationMapSnapshot.FromOccupancyGrid(msg);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < updateInterval) return;
        timer = 0f;

        // 맵은 자체 구독분 우선, 없으면 explorer 것이라도 사용 (하위호환)
        var snap = mapSnapshot
                   ?? (explorer != null ? explorer.MapSnapshot : null);
        if (snap == null) return;

        if (!ReferenceEquals(snap, lastSnapshot))
        {
            BuildBaseLayer(snap);
            lastSnapshot = snap;
        }

        RenderFrame(snap);
    }

    // ── 렌더 단계 ─────────────────────────────────────────────────

    /// <summary>점유 격자를 basePixels에 굽는다. 맵 데이터가 바뀔 때만 호출.</summary>
    void BuildBaseLayer(ExplorationMapSnapshot snap)
    {
        for (int py = 0; py < textureSize; py++)
        for (int px = 0; px < textureSize; px++)
        {
            int cx = px * snap.Width  / textureSize;
            int cy = py * snap.Height / textureSize;
            int v  = snap.Cell(cx, cy);

            basePixels[py * textureSize + px] =
                v < 0  ? UnknownColor  :
                v > 50 ? OccupiedColor : FreeColor;
        }
    }

    /// <summary>basePixels를 복사 후 동적 레이어를 순서대로 덮어씌운다.</summary>
    void RenderFrame(ExplorationMapSnapshot snap)
    {
        System.Array.Copy(basePixels, framePixels, framePixels.Length);

        DrawGasOverlay(snap);
        DrawMachines(snap);
        DrawPath(snap);
        DrawRobot(snap);

        mapTexture.SetPixels32(framePixels);
        mapTexture.Apply(false);
    }

    void DrawGasOverlay(ExplorationMapSnapshot snap)
    {
        foreach (var model in gasModels)
        {
            if (model == null || !model.isLeaking) continue;

            float invDanger = model.dangerThreshold > 0f ? 1f / model.dangerThreshold : 0f;

            for (int py = 0; py < textureSize; py++)
            for (int px = 0; px < textureSize; px++)
            {
                int cx = px * snap.Width  / textureSize;
                int cy = py * snap.Height / textureSize;

                // 월드 좌표 직접 계산 (CellToWorld 호출 생략해 성능 확보)
                float worldX = -((cy + 0.5f) * snap.Resolution + snap.Origin.y);
                float worldZ =  (cx + 0.5f) * snap.Resolution + snap.Origin.x;

                float conc  = model.GetConcentration(worldX, worldZ);
                float ratio = Mathf.Clamp01(conc * invDanger);
                if (ratio < 0.05f) continue;

                byte r = 255;
                byte g = (byte)Mathf.RoundToInt(Mathf.Lerp(230f, 0f, ratio));
                byte a = (byte)Mathf.RoundToInt(Mathf.Lerp(80f,  200f, ratio));

                int idx = py * textureSize + px;
                framePixels[idx] = BlendOver(framePixels[idx], new Color32(r, g, 0, a));
            }
        }
    }

    void DrawMachines(ExplorationMapSnapshot snap)
    {
        foreach (var m in machines)
        {
            if (m == null) continue;
            Vector2Int cell = snap.WorldToCell(m.transform.position);
            DrawDot(snap, cell, 3, GetMachineColor(m.Status));
        }
    }

    void DrawPath(ExplorationMapSnapshot snap)
    {
        // Frontier 탐색이 없거나 꺼져 있으면 경로 레이어는 생략 (미니맵 자체는 계속 동작)
        if (explorer == null || !explorer.IsActive) return;

        IReadOnlyList<Vector3> wps   = explorer.Waypoints;
        int                    wpIdx = explorer.WaypointIndex;

        if (wps == null || wps.Count < 2) return;

        // 이미 지나친 구간은 생략하고 남은 경로만 그린다
        for (int i = Mathf.Max(0, wpIdx); i < wps.Count - 1; i++)
        {
            DrawLine(snap, snap.WorldToCell(wps[i]), snap.WorldToCell(wps[i + 1]), PathColor);
        }

        // 현재 목표 웨이포인트 강조
        if (wpIdx < wps.Count)
            DrawDot(snap, snap.WorldToCell(wps[wpIdx]), 2, WpColor);
    }

    void DrawRobot(ExplorationMapSnapshot snap)
    {
        // robotTransform 우선, 없으면 explorer 트랜스폼 폴백
        Transform t = robotTransform != null
            ? robotTransform
            : (explorer != null ? explorer.transform : null);
        if (t == null) return;

        Vector2Int rc = snap.WorldToCell(t.position);
        DrawDot(snap, rc, 3, RobotColor);

        // 진행 방향 표시 (짧은 선)
        Vector3 tipWorld = t.position + t.forward * snap.Resolution * 6f;
        DrawLine(snap, rc, snap.WorldToCell(tipWorld), RobotColor);
    }

    // ── 드로우 헬퍼 ──────────────────────────────────────────────

    Color32 GetMachineColor(MachineHeat.HeatStatus s) => s switch
    {
        MachineHeat.HeatStatus.Caution => new Color32(255, 220, 0,   255),
        MachineHeat.HeatStatus.Warning => new Color32(255, 120, 0,   255),
        MachineHeat.HeatStatus.Danger  => new Color32(255, 40,  40,  255),
        _                              => new Color32(80,  140, 255, 255),
    };

    void DrawDot(ExplorationMapSnapshot snap, Vector2Int cell, int r, Color32 col)
    {
        int px = cell.x * textureSize / snap.Width;
        int py = cell.y * textureSize / snap.Height;
        for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
        {
            if (dx * dx + dy * dy > r * r) continue;
            SetPixel(px + dx, py + dy, col);
        }
    }

    void DrawLine(ExplorationMapSnapshot snap, Vector2Int a, Vector2Int b, Color32 col)
    {
        int ax = a.x * textureSize / snap.Width,  ay = a.y * textureSize / snap.Height;
        int bx = b.x * textureSize / snap.Width,  by = b.y * textureSize / snap.Height;
        int steps = Mathf.Max(Mathf.Abs(bx - ax), Mathf.Abs(by - ay));
        if (steps == 0) { SetPixel(ax, ay, col); return; }
        for (int i = 0; i <= steps; i++)
        {
            SetPixel(
                Mathf.RoundToInt(Mathf.Lerp(ax, bx, (float)i / steps)),
                Mathf.RoundToInt(Mathf.Lerp(ay, by, (float)i / steps)),
                col);
        }
    }

    void SetPixel(int x, int y, Color32 col)
    {
        if ((uint)x >= (uint)textureSize || (uint)y >= (uint)textureSize) return;
        framePixels[y * textureSize + x] = col;
    }

    static Color32 BlendOver(Color32 dst, Color32 src)
    {
        float a = src.a / 255f;
        return new Color32(
            (byte)(src.r * a + dst.r * (1f - a)),
            (byte)(src.g * a + dst.g * (1f - a)),
            (byte)(src.b * a + dst.b * (1f - a)),
            255);
    }
}
