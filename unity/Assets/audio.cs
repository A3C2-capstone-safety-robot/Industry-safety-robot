using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;        // robot_spot 드래그
    public Vector3 offset = new Vector3(0, 3, -5);
    public float smoothSpeed = 5f;

    void LateUpdate()
    {
        Vector3 desiredPos = target.position + target.rotation * offset;
        transform.position = Vector3.Lerp(transform.position, desiredPos, smoothSpeed * Time.deltaTime);
        transform.LookAt(target.position + Vector3.up * 1.5f);
    }
}