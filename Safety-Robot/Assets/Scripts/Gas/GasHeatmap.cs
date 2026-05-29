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

    [Header("해상도")]
    public int resolution = 128;                   // 텍스처 해상도 (128x128)
    public float updateInterval = 0.5f;            // 업데이트 주기 (초)

    [Header("색상 범위")]
    public float minConcentration = 1f;            // 이 이하는 투명
    public float maxConcentration = 100f;          // 이 이상은 최대 색상

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
        CreateHeatmapPlane();
        colorBuffer = new Color[resolution * resolution];
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
        float sampleY = centerPosition.y + 0.5f; // 바닥에서 0.5m 높이에서 샘플링

        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float worldX = startX + x * step;
                float worldZ = startZ + y * step;

                // 모든 활성 누출원의 농도 합산
                float totalConc = 0f;
                for (int i = 0; i < allPlumes.Length; i++)
                {
                    if (allPlumes[i] == null || !allPlumes[i].isLeaking) continue;
                    totalConc += allPlumes[i].GetConcentration(worldX, sampleY, worldZ);
                }

                // 농도 → 색상 변환
                colorBuffer[y * resolution + x] = ConcentrationToColor(totalConc);
            }
        }

        heatmapTexture.SetPixels(colorBuffer);
        heatmapTexture.Apply();
    }

    // 농도 값을 히트맵 색상으로 변환
    // 낮음(파랑) → 중간(초록→노랑) → 높음(빨강)
    Color ConcentrationToColor(float concentration)
    {
        if (concentration < minConcentration)
            return Color.clear; // 투명

        // 0~1로 정규화
        float t = Mathf.Clamp01((concentration - minConcentration) / (maxConcentration - minConcentration));

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
