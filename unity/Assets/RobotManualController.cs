using UnityEngine;

public class RobotManualController : MonoBehaviour
{

    public float moveSpeed = 2.0f;
    public float rotateSpeed = 80.0f;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
        void Update()
    {
        if (Input.GetKey(KeyCode.W))
            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.S))
            transform.Translate(Vector3.back * moveSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.A))
            transform.Rotate(Vector3.up * -rotateSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.D))
            transform.Rotate(Vector3.up * rotateSpeed * Time.deltaTime);
    }
}


/* check if input works */
// using UnityEngine;

// public class RobotManualController : MonoBehaviour
// {
//     public float speed = 2f;

//     void Update()
//     {
//         transform.position += Vector3.forward * speed * Time.deltaTime;
//     }
// }