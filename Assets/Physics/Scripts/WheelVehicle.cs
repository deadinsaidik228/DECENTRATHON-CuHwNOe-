using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class WheelVehicle : MonoBehaviour
{
    [SerializeField] AnimationCurve turnInputCurve = AnimationCurve.Linear(-1.0f, -1.0f, 1.0f, 1.0f);

    [Header("Wheels")]
    [SerializeField] WheelCollider[] driveWheel;
    public WheelCollider[] DriveWheel { get { return driveWheel; } }
    [SerializeField] WheelCollider[] turnWheel;

    public WheelCollider[] TurnWheel { get { return turnWheel; } }
    bool isGrounded = false;
    int lastGroundCheck = 0;
    public bool IsGrounded
    {
        get
        {
            if (lastGroundCheck == Time.frameCount)
                return isGrounded;

            lastGroundCheck = Time.frameCount;
            isGrounded = true;
            foreach (WheelCollider wheel in wheels)
            {
                if (!wheel.gameObject.activeSelf || !wheel.isGrounded)
                    isGrounded = false;
            }
            return isGrounded;
        }
    }

    [Header("Behaviour")]
    [SerializeField] AnimationCurve motorTorque = new AnimationCurve(new Keyframe(0, 200), new Keyframe(50, 300), new Keyframe(200, 0));

    [Range(2, 16)]
    [SerializeField] float diffGearing = 4.0f;
    public float DiffGearing { get { return diffGearing; } set { diffGearing = value; } }
    [SerializeField] float brakeForce = 1500.0f;
    public float BrakeForce { get { return brakeForce; } set { brakeForce = value; } }

    [Range(0f, 50.0f)]
    [SerializeField] float steerAngle = 30.0f;
    public float SteerAngle { get { return steerAngle; } set { steerAngle = Mathf.Clamp(value, 0.0f, 50.0f); } }
    [Range(0.001f, 1.0f)]
    [SerializeField] float steerSpeed = 0.2f;
    public float SteerSpeed { get { return steerSpeed; } set { steerSpeed = Mathf.Clamp(value, 0.001f, 1.0f); } }


    [Range(0.0f, 2f)]
    [SerializeField] float driftIntensity = 1f;
    public float DriftIntensity { get { return driftIntensity; } set { driftIntensity = Mathf.Clamp(value, 0.0f, 2.0f); } }

    Vector3 spawnPosition;
    Quaternion spawnRotation;
    [SerializeField] Transform centerOfMass;
    [Range(0.5f, 10f)]
    [SerializeField] float downforce = 1.0f;
    public float Downforce { get { return downforce; } set { downforce = Mathf.Clamp(value, 0, 5); } }

    float steering;
    public float Steering { get { return steering; } set { steering = Mathf.Clamp(value, -1f, 1f); } }

    float throttle;
    public float Throttle { get { return throttle; } set { throttle = Mathf.Clamp(value, -1f, 1f); } }

    [SerializeField] bool handbrake;
    public bool Handbrake { get { return handbrake; } set { handbrake = value; } }

    [HideInInspector] public bool allowDrift = true;
    bool drift;
    public bool Drift { get { return drift; } set { drift = value; } }

    [SerializeField] float speed = 0.0f;
    public float Speed { get { return speed; } }


    Rigidbody _rb;
    WheelCollider[] wheels;
    [SerializeField]  private Vector2 m_movement;

    void Start()
    {

        _rb = GetComponent<Rigidbody>();
        spawnPosition = transform.position;
        spawnRotation = transform.rotation;

        if (_rb != null && centerOfMass != null)
        {
            _rb.centerOfMass = centerOfMass.localPosition;
        }

        wheels = GetComponentsInChildren<WheelCollider>();

        foreach (WheelCollider wheel in wheels)
        {
            wheel.motorTorque = 0.0001f;
        }
    }


    public void Move(InputAction.CallbackContext context)
    {

        m_movement = context.ReadValue<Vector2>();

    }
    public float GetSteeringAngle()
    {
        return (turnWheel[0].steerAngle / steerAngle);
    }
    public void AgentMove(Vector2 _move)
    {

        m_movement = _move;
    }
    public float GetTorque()
    {
        return (driveWheel[0].motorTorque / 602f);
    }
    void FixedUpdate()
    {
        Debug.Log("torque = " + driveWheel[0].motorTorque / 602f);
        Debug.Log("Steering angle = " + turnWheel[0].steerAngle / steerAngle);
        speed = transform.InverseTransformDirection(_rb.velocity).z * 3.6f;


        throttle = m_movement.y;
        steering = turnInputCurve.Evaluate(m_movement.x) * steerAngle;


        foreach (WheelCollider wheel in turnWheel)
        {
            wheel.steerAngle = Mathf.Lerp(wheel.steerAngle, steering, steerSpeed);
        }

        foreach (WheelCollider wheel in wheels)
        {
            wheel.brakeTorque = 0;
        }
        if (handbrake)
        {
            foreach (WheelCollider wheel in wheels)
            {
                wheel.motorTorque = 0.0001f;
                wheel.brakeTorque = brakeForce;
            }
        }else if (Mathf.Abs(speed) < 65 && Mathf.Sign(speed) == Mathf.Sign(throttle))
        {
            foreach (WheelCollider wheel in driveWheel)
            {
                wheel.motorTorque = throttle * motorTorque.Evaluate(speed) * diffGearing / driveWheel.Length * 2f;
            }
        }else
        {
            foreach (WheelCollider wheel in wheels)
            {
                wheel.brakeTorque = Mathf.Abs(throttle) * brakeForce;
            }
        }
        if (drift && allowDrift)
        {
            Vector3 driftForce = -transform.right;
            driftForce.y = 0.0f;
            driftForce.Normalize();

            if (steering != 0)
                driftForce *= _rb.mass * speed / 7f * throttle * steering / steerAngle;
            Vector3 driftTorque = transform.up * 0.1f * steering / steerAngle;


            _rb.AddForce(driftForce * driftIntensity, ForceMode.Force);
            _rb.AddTorque(driftTorque * driftIntensity, ForceMode.VelocityChange);
        }
        _rb.AddForce(-transform.up * speed * downforce);
    }
    public void ResetPos()
    {
        transform.position = spawnPosition;
        transform.rotation = spawnRotation;

        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
    }

    public void toogleHandbrake(bool h)
    {
        handbrake = h;
    }

}
