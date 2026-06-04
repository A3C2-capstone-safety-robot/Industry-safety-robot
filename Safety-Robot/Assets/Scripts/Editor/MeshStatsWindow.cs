using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// 씬의 모든 메시를 트라이앵글 수 기준으로 정렬해서 보여주는 창.
/// 메뉴: Tools > Mesh Stats
/// </summary>
public class MeshStatsWindow : EditorWindow
{
    class Entry
    {
        public Mesh mesh;
        public int trisPerInstance;
        public int instanceCount;
        public int totalTris;
        public GameObject sample; // 클릭하면 씬에서 찾아갈 대표 오브젝트
    }

    List<Entry> entries = new List<Entry>();
    Vector2 scroll;
    string totalSummary = "";

    [MenuItem("Tools/Mesh Stats")]
    static void Open() => GetWindow<MeshStatsWindow>("Mesh Stats");

    void OnGUI()
    {
        if (GUILayout.Button("씬 스캔", GUILayout.Height(30)))
            Scan();

        EditorGUILayout.LabelField(totalSummary, EditorStyles.boldLabel);
        EditorGUILayout.Space(4);

        if (entries.Count == 0) return;

        // 헤더
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("메시 이름", EditorStyles.miniBoldLabel, GUILayout.MinWidth(150));
        EditorGUILayout.LabelField("개당 Tris", EditorStyles.miniBoldLabel, GUILayout.Width(80));
        EditorGUILayout.LabelField("개수", EditorStyles.miniBoldLabel, GUILayout.Width(60));
        EditorGUILayout.LabelField("총 Tris", EditorStyles.miniBoldLabel, GUILayout.Width(90));
        EditorGUILayout.EndHorizontal();

        scroll = EditorGUILayout.BeginScrollView(scroll);
        foreach (var e in entries.Take(300)) // 상위 300개만 표시
        {
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(e.mesh.name, EditorStyles.linkLabel, GUILayout.MinWidth(150)))
            {
                Selection.activeGameObject = e.sample;
                EditorGUIUtility.PingObject(e.sample);
            }
            EditorGUILayout.LabelField(e.trisPerInstance.ToString("N0"), GUILayout.Width(80));
            EditorGUILayout.LabelField(e.instanceCount.ToString("N0"), GUILayout.Width(60));
            EditorGUILayout.LabelField(e.totalTris.ToString("N0"), GUILayout.Width(90));
            EditorGUILayout.EndHorizontal();
        }
        EditorGUILayout.EndScrollView();
    }

    void Scan()
    {
        var map = new Dictionary<Mesh, Entry>();

        foreach (var mf in FindObjectsByType<MeshFilter>(FindObjectsSortMode.None))
            Add(map, mf.sharedMesh, mf.gameObject);

        foreach (var smr in FindObjectsByType<SkinnedMeshRenderer>(FindObjectsSortMode.None))
            Add(map, smr.sharedMesh, smr.gameObject);

        entries = map.Values.OrderByDescending(e => e.totalTris).ToList();

        long grandTotal = entries.Sum(e => (long)e.totalTris);
        int objCount = entries.Sum(e => e.instanceCount);
        totalSummary = $"오브젝트 {objCount:N0}개 / 고유 메시 {entries.Count:N0}종 / 총 {grandTotal:N0} tris";
    }

    static void Add(Dictionary<Mesh, Entry> map, Mesh mesh, GameObject go)
    {
        if (mesh == null) return;
        if (!map.TryGetValue(mesh, out var e))
        {
            e = new Entry
            {
                mesh = mesh,
                trisPerInstance = (int)(mesh.GetIndexCount(0) / 3), // 서브메시 0 기준 근사
                sample = go
            };
            // 전체 서브메시 합산
            int tris = 0;
            for (int i = 0; i < mesh.subMeshCount; i++)
                tris += (int)(mesh.GetIndexCount(i) / 3);
            e.trisPerInstance = tris;
            map[mesh] = e;
        }
        e.instanceCount++;
        e.totalTris = e.trisPerInstance * e.instanceCount;
    }
}
