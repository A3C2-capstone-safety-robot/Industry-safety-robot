using UnityEngine;
using UnityEngine.InputSystem;

public class SpotController : MonoBehaviour
{
    public float moveSpeed = 3f;
    public float rotateSpeed = 100f;
    private Animator anim;

    void Start()
    {
        anim = GetComponent<Animator>();
    }

    void Update()
    {
        float h = Keyboard.current.aKey.isPressed ? -1f :
                  Keyboard.current.dKey.isPressed ? 1f : 0f;
        float v = Keyboard.current.sKey.isPressed ? -1f :
                  Keyboard.current.wKey.isPressed ? 1f : 0f;

        transform.Translate(0, 0, v * moveSpeed * Time.deltaTime);
        transform.Rotate(0, h * rotateSpeed * Time.deltaTime, 0);

        if (anim != null)
            anim.SetFloat("Speed", Mathf.Abs(v));
    }
}