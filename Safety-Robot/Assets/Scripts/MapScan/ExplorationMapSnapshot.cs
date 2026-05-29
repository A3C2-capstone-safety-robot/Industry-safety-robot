using UnityEngine;
using RosMessageTypes.Nav;

/// <summary>
/// SLAM occupancy grid를 Unity 쪽에서 다루기 쉽게 감싼 읽기 전용 맵 스냅샷.
/// Frontier 탐색기, 경로계획기, 디버그 도구가 동일한 좌표 변환 규칙을 공유하도록 만든다.
/// </summary>
public sealed class ExplorationMapSnapshot
{
    readonly int[] grid;

    public int Width { get; }
    public int Height { get; }
    public float Resolution { get; }
    public Vector2 Origin { get; }

    public ExplorationMapSnapshot(int width, int height, float resolution, Vector2 origin, int[] cells)
    {
        Width = width;
        Height = height;
        Resolution = resolution;
        Origin = origin;
        grid = cells;
    }

    public static ExplorationMapSnapshot FromOccupancyGrid(OccupancyGridMsg msg)
    {
        int[] cells = new int[msg.data.Length];
        for (int i = 0; i < msg.data.Length; i++)
            cells[i] = msg.data[i];

        return new ExplorationMapSnapshot(
            (int)msg.info.width,
            (int)msg.info.height,
            msg.info.resolution,
            new Vector2((float)msg.info.origin.position.x, (float)msg.info.origin.position.y),
            cells);
    }

    public int Cell(int cx, int cy)
    {
        if (cx < 0 || cy < 0 || cx >= Width || cy >= Height) return -1;
        return grid[cy * Width + cx];
    }

    public Vector2Int WorldToCell(Vector3 world)
    {
        return new Vector2Int(
            Mathf.FloorToInt((world.z - Origin.x) / Resolution),
            Mathf.FloorToInt((-world.x - Origin.y) / Resolution));
    }

    public Vector3 CellToWorld(int cx, int cy, float worldY)
    {
        float rosX = (cx + 0.5f) * Resolution + Origin.x;
        float rosY = (cy + 0.5f) * Resolution + Origin.y;
        return new Vector3(-rosY, worldY, rosX);
    }

    public Vector2Int CellToBlock(Vector2Int cell, int blockSize)
    {
        return new Vector2Int(FloorDiv(cell.x, blockSize), FloorDiv(cell.y, blockSize));
    }

    public Vector2Int BlockToCenter(Vector2Int block, int blockSize)
    {
        int cx = Mathf.Clamp(block.x * blockSize + blockSize / 2, 0, Width > 0 ? Width - 1 : 0);
        int cy = Mathf.Clamp(block.y * blockSize + blockSize / 2, 0, Height > 0 ? Height - 1 : 0);
        return new Vector2Int(cx, cy);
    }

    public int BlockWidth(int blockSize)
    {
        return (Width + blockSize - 1) / blockSize;
    }

    public int BlockHeight(int blockSize)
    {
        return (Height + blockSize - 1) / blockSize;
    }

    static int FloorDiv(int a, int b)
    {
        return (a >= 0) ? a / b : (a - b + 1) / b;
    }
}
