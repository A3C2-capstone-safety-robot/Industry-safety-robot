using UnityEngine;

public class PlayerCamera : MonoBehaviour
{
    [Header("이동")]
    public float moveSpeed = 5f;
    public float fastMoveMultiplier = 2.5f;

    [Header("시점")]
    public float lookSpeed = 2f;
    public bool rotateOnlyWhileRightMouseHeld = true;

    private float rotX = 0f;
    private float rotY = 0f;

    void Start()
    {
        rotX = transform.eulerAngles.y;
        rotY = transform.eulerAngles.x;
    }

    void Update()
    {
        bool canRotate = !rotateOnlyWhileRightMouseHeld || Input.GetMouseButton(1);
        if (canRotate)
        {
            rotX += Input.GetAxis("Mouse X") * lookSpeed;
            rotY -= Input.GetAxis("Mouse Y") * lookSpeed;
            rotY = Mathf.Clamp(rotY, -80f, 80f);
            transform.rotation = Quaternion.Euler(rotY, rotX, 0f);
        }

        float h = Input.GetAxis("Horizontal");
        float v = Input.GetAxis("Vertical");
        float up = 0f;
        if (Input.GetKey(KeyCode.E)) up = 1f;
        if (Input.GetKey(KeyCode.Q)) up = -1f;

        float speed = moveSpeed;
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            speed *= fastMoveMultiplier;

        Vector3 move = transform.right * h + transform.forward * v + Vector3.up * up;
        transform.position += move * speed * Time.deltaTime;
    }
}
