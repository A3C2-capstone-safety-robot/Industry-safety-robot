using UnityEngine;

public class RobotWalkController : MonoBehaviour
{
    public Animator animator;
    public ManualTeachController manualTeachController;
    public string isMovingParameter = "IsMoving";
    public float movingThreshold = 0.02f;
    public float stopDelay = 0.15f;

    private Vector3 previousPosition;
    private int isMovingHash;
    private float lastMovingTime;
    private bool hasMovingParameter;

    void Start()
    {
        if (animator == null)
            animator = GetComponentInChildren<Animator>();
        if (manualTeachController == null)
            manualTeachController = GetComponent<ManualTeachController>();

        isMovingHash = Animator.StringToHash(isMovingParameter);
        hasMovingParameter = HasAnimatorBool(animator, isMovingParameter);
        previousPosition = transform.position;
    }

    void Update()
    {
        if (animator == null)
            return;
        if (!hasMovingParameter)
            return;

        Vector3 delta = transform.position - previousPosition;
        delta.y = 0f;

        float speed = delta.magnitude / Mathf.Max(Time.deltaTime, 0.0001f);
        bool manualMoving = manualTeachController != null &&
                            manualTeachController.enabled &&
                            manualTeachController.IsCommandedMoving;

        if (manualMoving || speed > movingThreshold)
            lastMovingTime = Time.time;

        bool isMoving = Time.time - lastMovingTime <= stopDelay;

        animator.SetBool(isMovingHash, isMoving);
        previousPosition = transform.position;
    }

    private bool HasAnimatorBool(Animator targetAnimator, string parameterName)
    {
        if (targetAnimator == null)
            return false;

        foreach (AnimatorControllerParameter parameter in targetAnimator.parameters)
        {
            if (parameter.type == AnimatorControllerParameterType.Bool &&
                parameter.name == parameterName)
                return true;
        }

        Debug.LogWarning($"[RobotWalkController] Animator에 Bool 파라미터 '{parameterName}'가 없습니다.", this);
        return false;
    }
}
