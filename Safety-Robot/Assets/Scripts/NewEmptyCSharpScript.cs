using UnityEngine;

public class PlayerCamera : MonoBehaviour
{
    [Header("이동")]
    public float moveSpeed = 5f;

    [Header("시점")]
    public float lookSpeed = 2f;

    private float rotX = 0f;
    private float rotY = 0f;
    private bool isLooking = false;

    void Start()
    {
        rotX = transform.eulerAngles.y;
        rotY = transform.eulerAngles.x;
    }

    void Update()
    {
        // 마우스 우클릭 누른 동안만 시점 회전
        if (Input.GetMouseButton(1))
        {
            rotX += Input.GetAxis("Mouse X") * lookSpeed;
            rotY -= Input.GetAxis("Mouse Y") * lookSpeed;
            rotY = Mathf.Clamp(rotY, -80f, 80f);
            transform.rotation = Quaternion.Euler(rotY, rotX, 0f);
        }

        // WASD 이동
        float h = Input.GetAxis("Horizontal");
        float v = Input.GetAxis("Vertical");
        float up = 0f;
        if (Input.GetKey(KeyCode.E)) up = 1f;
        if (Input.GetKey(KeyCode.Q)) up = -1f;

        Vector3 move = transform.right * h + transform.forward * v + Vector3.up * up;
        transform.position += move * moveSpeed * Time.deltaTime;
    }
}