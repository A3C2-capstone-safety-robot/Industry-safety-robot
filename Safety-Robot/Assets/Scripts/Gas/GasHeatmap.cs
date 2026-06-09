// GasHeatmap.cs
// 가스 농도 2D 히트맵 시각화
// 씬 바닥에 투명 평면으로 실시간 농도 분포를 표시
using UnityEngine;

public class GasHeatmap : MonoBehaviour
{
    [Header("히트맵 영역")]
    public Vector3 centerPosition = Vector3.zero;  // 히트맵 중심 좌표
    public float areaSize = 40f;                   // 히트맵 한 변 크기 (m)
    public float heightOffset = 0.05f;             // 바닥 위 높이 (겹침 방지)

    [Header("자동 영역 맞춤")]
    [Tooltip("씬의 모든 GasLeakPoint를 포함하도록 중심/크기 자동 조정 (구석 누출원 누락 방지)")]
    public bool autoFitToLeakSources = true;
    public float fitMargin = 12f;                  // 가장 바깥 누출원 둘레 여유 (m)

    [Header("해상도")]
    public int resolution = 128;                   // 텍스처 해상도 (128x128)
    public float updateInterval = 0.5f;            // 업데이트 주기 (초)

    [Header("색상 범위 — 가스별 위험도(dangerThreshold 대비 비율) 기준")]
    [Tooltip("위험도 = 농도 ÷ 해당 가스의 dangerThreshold. 1.0 = 위험 임계 도달")]
    public float minHazardRatio = 0.05f;           // 이 이하는 투명 (위험 임계의 5%)
    public float maxHazardRatio = 1.5f;            // 이 이상은 최대 색상(빨강)

    [Header("표시 설정")]
    public bool showHeatmap = true;
    public float opacity = 0.5f;                   // 전체 투명도 (0~1)
    public KeyCode toggleKey = KeyCode.H;          // H키로 표시/숨김

    // 내부
    private GaussianPlumeModel[] allPlumes;
    private Texture2D heatmapTexture;
    private GameObject heatmapPlane;
    private MeshRenderer planeRenderer;
    private Material heatmapMaterial;
    private float timer = 0f;
    private Color[] colorBuffer;

    void Start()
    {
        allPlumes = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
        if (autoFitToLeakSources) AutoFitArea();
        CreateHeatmapPlane();
        colorBuffer = new Color[resolution * resolution];
    }

    // 씬의 모든 누출원을 포함하도록 히트맵 중심/크기 자동 조정
    void AutoFitArea()
    {
        bool any = false;
        float minX = 0f, maxX = 0f, minZ = 0f, maxZ = 0f;

        for (int i = 0; i < allPlumes.Length; i++)
        {
            if (allPlumes[i] == null || allPlumes[i].leakSource == null) continue;
            Vector3 p = allPlumes[i].leakSource.position;
            if (!any) { minX = maxX = p.x; minZ = maxZ = p.z; any = true; }
            else
            {
                minX = Mathf.Min(minX, p.x); maxX = Mathf.Max(maxX, p.x);
                minZ = Mathf.Min(minZ, p.z); maxZ = Mathf.Max(maxZ, p.z);
            }
        }
        if (!any) return;

        centerPosition = new Vector3(
            (minX + maxX) * 0.5f, centerPosition.y, (minZ + maxZ) * 0.5f);
        areaSize = Mathf.Max(maxX - minX, maxZ - minZ) + fitMargin * 2f;

        Debug.Log($"[히트맵] 자동 맞춤 — 중심:({centerPosition.x:F1},{centerPosition.z:F1}), " +
                  $"크기:{areaSize:F0}m (누출원 {allPlumes.Length}개 포함)");
    }

    void Update()
    {
        // H키로 토글
        if (Input.GetKeyDown(toggleKey))
        {
            showHeatmap = !showHeatmap;
            heatmapPlane.SetActive(showHeatmap);
        }

        if (!showHeatmap) return;

        // 주기적 업데이트 (매 프레임은 너무 무거움)
        timer += Time.deltaTime;
        if (timer >= updateInterval)
        {
            timer = 0f;

            // 플룸 목록 갱신 (새 누출원이 추가될 수 있음)
            allPlumes = FindObjectsByType<GaussianPlumeModel>(FindObjectsSortMode.None);
            UpdateHeatmap();
        }
    }

    // 히트맵 표시용 평면 생성
    void CreateHeatmapPlane()
    {
        heatmapPlane = GameObject.CreatePrimitive(PrimitiveType.Quad);
        heatmapPlane.name = "GasHeatmapPlane";
        heatmapPlane.transform.SetParent(transform);

        // 바닥에 수평으로 놓기 (Quad는 기본이 수직이라 회전 필요)
        heatmapPlane.transform.position = new Vector3(
            centerPosition.x, centerPosition.y + heightOffset, centerPosition.z);
        heatmapPlane.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        heatmapPlane.transform.localScale = new Vector3(areaSize, areaSize, 1f);

        // 콜라이더 제거 (히트맵이 물리에 영향 주면 안 됨)
        Destroy(heatmapPlane.GetComponent<Collider>());

        // 텍스처 생성
        heatmapTexture = new Texture2D(resolution, resolution, TextureFormat.RGBA32, false);
        heatmapTexture.filterMode = FilterMode.Bilinear;
        heatmapTexture.wrapMode = TextureWrapMode.Clamp;

        // 투명 머티리얼 생성
        heatmapMaterial = new Material(Shader.Find("Sprites/Default"));
        heatmapMaterial.mainTexture = heatmapTexture;

        planeRenderer = heatmapPlane.GetComponent<MeshRenderer>();
        planeRenderer.material = heatmapMaterial;
        planeRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        planeRenderer.receiveShadows = false;
    }

    // 히트맵 텍스처 업데이트
    void UpdateHeatmap()
    {
        float halfSize = areaSize * 0.5f;
        float startX = centerPosition.x - halfSize;
        float startZ = centerPosition.z - halfSize;
        float step = areaSize / resolution;
        // ★ 높이 무시 2D 농도 사용 — 센서(VirtualGasSensor)와 동일 기준.
        //   3D를 쓰면 LNG처럼 떠오르는 가스는 바닥 샘플링 높이에서 사라져
        //   '로봇은 추적 중인데 히트맵엔 안 보이는' 불일치 발생.

        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float worldX = startX + x * step;
                float worldZ = startZ + y * step;

                // 모든 활성 누출원의 '위험도' 합산 — 가스별 dangerThreshold로 정규화
                // (H2S 20ppm과 LNG 100ppm이 같은 빨강 = 같은 위험 수준)
                float totalHazard = 0f;
                for (int i = 0; i < allPlumes.Length; i++)
                {
                    if (allPlumes[i] == null || !allPlumes[i].isLeaking) continue;
                    float conc = allPlumes[i].GetConcentration(worldX, worldZ);
                    totalHazard += conc / Mathf.Max(1f, allPlumes[i].dangerThreshold);
                }

                // 위험도 → 색상 변환
                colorBuffer[y * resolution + x] = HazardToColor(totalHazard);
            }
        }

        heatmapTexture.SetPixels(colorBuffer);
        heatmapTexture.Apply();
    }

    // 위험도(danger 임계 대비 비율)를 히트맵 색상으로 변환
    // 낮음(파랑) → 중간(초록→노랑) → 높음(빨강=위험 임계 초과)
    Color HazardToColor(float hazardRatio)
    {
        if (hazardRatio < minHazardRatio)
            return Color.clear; // 투명

        // 0~1로 정규화
        float t = Mathf.Clamp01((hazardRatio - minHazardRatio) / (maxHazardRatio - minHazardRatio));

        Color color;

        if (t < 0.25f)
        {
            // 파랑 → 시안
            float lt = t / 0.25f;
            color = Color.Lerp(new Color(0f, 0f, 1f), new Color(0f, 1f, 1f), lt);
        }
        else if (t < 0.5f)
        {
            // 시안 → 초록
            float lt = (t - 0.25f) / 0.25f;
            color = Color.Lerp(new Color(0f, 1f, 1f), new Color(0f, 1f, 0f), lt);
        }
        else if (t < 0.75f)
        {
            // 초록 → 노랑
            float lt = (t - 0.5f) / 0.25f;
            color = Color.Lerp(new Color(0f, 1f, 0f), new Color(1f, 1f, 0f), lt);
        }
        else
        {
            // 노랑 → 빨강
            float lt = (t - 0.75f) / 0.25f;
            color = Color.Lerp(new Color(1f, 1f, 0f), new Color(1f, 0f, 0f), lt);
        }

        // 농도에 따라 투명도 조절 (낮으면 더 투명)
        color.a = Mathf.Lerp(0.1f, opacity, t);

        return color;
    }

    // Inspector에서 영역 시각화
    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
        Gizmos.DrawWireCube(
            new Vector3(centerPosition.x, centerPosition.y + heightOffset, centerPosition.z),
            new Vector3(areaSize, 0.1f, areaSize));
    }
}
