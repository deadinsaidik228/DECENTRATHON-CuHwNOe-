using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarAgent : Agent
{

    [SerializeField] private bool m_allGrounded = false;
    [SerializeField] private float m_currentReward;
    [SerializeField] private float m_distanceToTarget;
    [SerializeField] private float m_prevDistanceToTarget;
    [SerializeField] private int m_obstacleHit;
    [SerializeField] private int m_prevObstacleHit;
    [SerializeField] private int m_nextCheckpointNumber;
    [SerializeField] private GameObject m_road;
    [SerializeField] private GameObject m_obstacles;
    [SerializeField] private Transform m_target;
    [SerializeField] private Vector3 m_dirToTarget;
    [SerializeField] private Vector3 m_velocity;
    [SerializeField] private Vector3 m_angularVelocity;

    private WheelVehicle m_carController;
    private int m_steps;
    private int m_deadCounter;
    private ObstacleGenerator m_obstacleGen;
    private Rigidbody m_carRigidbody;
    private RoadGenerator m_roadGen;
    private Transform m_nextCheckpoint;
    private Vector2 m_move;
    private Vector3 m_checkpointPos;
    private WheelCollider[] m_wheelColliders;
    private WheelHit m_out;

    public override void Initialize()
    {
        m_roadGen = m_road.GetComponent<RoadGenerator>();
        m_carController = GetComponent<WheelVehicle>();
        m_carRigidbody = GetComponentInChildren<Rigidbody>();
        m_wheelColliders = GetComponentsInChildren<WheelCollider>();
        m_obstacleGen = m_obstacles.GetComponent<ObstacleGenerator>();
    }

    public override void OnEpisodeBegin()
    {
        RoadAndObstacleReset();
        ResetAll();
        PrivateVariableReset();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        m_move.x = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        m_move.y = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        m_carController.AgentMove(m_move);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var _actionsOut = actionsOut.ContinuousActions;
        _actionsOut[0] = m_move.x;
        _actionsOut[1] = m_move.y;
        m_carController.AgentMove(m_move);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        m_velocity = transform.InverseTransformDirection(m_carRigidbody.velocity) / 20f;
        sensor.AddObservation(new Vector2(m_velocity.x, m_velocity.z)); 
        sensor.AddObservation(m_distanceToTarget / 30f); 
        sensor.AddObservation(new Vector2(transform.localPosition.x / 500f, transform.localPosition.z / 500f)); // vec2

        sensor.AddObservation(m_carController.GetTorque()); 
        sensor.AddObservation(m_carController.GetSteeringAngle()); 


        m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;

        sensor.AddObservation(Vector3.Dot(transform.forward, m_dirToTarget)); 

        m_angularVelocity = transform.InverseTransformDirection(m_carRigidbody.angularVelocity) / 3f;
        sensor.AddObservation(m_angularVelocity.y); 
    }

    private void RoadAndObstacleReset()
    {
        float _rand = UnityEngine.Random.Range(-1f, 1f);

        m_roadGen.GenTrack((int)(Mathf.Sign(_rand) * Mathf.Ceil(Mathf.Abs(_rand))));

        m_obstacleGen.obstacleState = 1;
        m_obstacleGen.obstacleDensity = UnityEngine.Random.Range(0.6f, 1.1f);
        m_obstacleGen.ObstacleStateChange();
    }

    private void PrivateVariableReset()
    {
        m_steps = 0;

        m_carController.AgentMove(Vector2Int.zero);

        m_currentReward = 0;

        m_obstacleHit = 0;
        m_prevObstacleHit = 0;

        m_deadCounter = 0;

        m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
        m_prevDistanceToTarget = m_distanceToTarget;
    }
    private void ResetAll()
    {

        m_nextCheckpoint = m_roadGen.waypoints[0];
        m_nextCheckpointNumber = 1;

        m_carRigidbody.velocity = Vector3.zero;
        m_carRigidbody.angularVelocity = Vector3.zero;

        foreach (WheelCollider tempcol in m_wheelColliders)
        {
            tempcol.brakeTorque = Mathf.Infinity;
        }

        transform.localPosition = ((m_roadGen.vertices[4] + m_roadGen.vertices[5] + m_roadGen.vertices[6] + m_roadGen.vertices[7]) / 4f);

        transform.localRotation = Quaternion.LookRotation((((m_nextCheckpoint.localPosition) - new Vector3(0, m_roadGen.halfRoadWidth, 0)) - transform.localPosition).normalized, Vector3.up);

        transform.localPosition += new Vector3(0, 0.8f, 0);

        transform.localPosition += transform.right * UnityEngine.Random.Range((-m_roadGen.halfRoadWidth + 2f), (m_roadGen.halfRoadWidth - 2f));

        transform.Rotate(new Vector3(0f, UnityEngine.Random.Range(-45f, 45f), 0f));

        m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));

        m_target.transform.localPosition = ((m_roadGen.vertices[m_roadGen.vertices.Count - 1] + m_roadGen.vertices[m_roadGen.vertices.Count - 2] + m_roadGen.vertices[m_roadGen.vertices.Count - 3] + m_roadGen.vertices[m_roadGen.vertices.Count - 4]) / 4f);
        m_target.transform.localPosition += new Vector3(0, 0.5f, 0);
    }

    private void OnCollisionStay(Collision collision)
    {

        if (collision.collider.CompareTag("Obstacle"))
        {
            m_currentReward += -0.01f;
            AddReward(-0.01f);
        }
    }
        private void OnCollisionEnter(Collision other)
    {
        if (other.collider.CompareTag("Obstacle"))
        {
            m_currentReward += -1f * (m_obstacleHit + 1) * (m_currentReward / 10f);
            AddReward(-1f * (m_obstacleHit + 1) * (m_currentReward / 10f));

            m_obstacleHit++;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Finish"))
        {
            if (m_roadGen.waypoints.Count < m_nextCheckpointNumber)
            {
                m_currentReward += 1f + ((30f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
                AddReward(1f + ((30f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                EndEpisode();
            }else
            {
                NextEpisode(-1f);
            }
        }

        else if (other.CompareTag("Waypoint"))
        {

            if (other.gameObject.transform == m_nextCheckpoint)
            {
                m_currentReward += (0.5f + ((20f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                AddReward(0.5f + ((20f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));

                m_prevObstacleHit = m_obstacleHit;
                NewCheckpoint();
            }else
            {
                m_currentReward += (-1f);
                AddReward(-1f);
            }
        }else if (other.CompareTag("Respawn"))
        {
            NextEpisode(-1f);
        }
    }

    private void NewCheckpoint()
    {
        if (m_roadGen.waypoints.Count > m_nextCheckpointNumber)
        {
            m_nextCheckpoint = m_roadGen.waypoints[m_nextCheckpointNumber];
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber++;

            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }else
        {
            m_nextCheckpoint = m_target.transform;
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber++;

            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }
    }

    private void CheckMovement()
    {
        CheckDistanceFromCheckpoint();

        if (m_carRigidbody.velocity.magnitude > 1f)
        { 
            m_currentReward += (0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f));
            AddReward((0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f)));
        }
        if (m_carRigidbody.velocity.magnitude < 0.5f || !m_allGrounded)
        {
            m_deadCounter++;
        }

        if (m_carRigidbody.velocity.magnitude > 0.5f && m_allGrounded)
        {
            m_deadCounter = 0;
        }

        if (m_deadCounter >= 500)
        {
            m_currentReward += -0.001f;
            AddReward(-0.001f);
        }
    }
        private void CheckDistanceFromCheckpoint()
    {
        m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
        if (m_steps % 50 == 0)
        {
            //Debug.Log("Distance reward = " + (m_prevDistanceToTarget - m_distanceToTarget) / 25f);

            m_currentReward += (m_prevDistanceToTarget - m_distanceToTarget) / 25f;
            AddReward((m_prevDistanceToTarget - m_distanceToTarget) / 25f);

            m_prevDistanceToTarget = m_distanceToTarget;
        }
    }
    private void CheckGrounded()
    {
        foreach (WheelCollider tempcol in m_wheelColliders)
        {
            if (transform.position.y >= 1f || transform.position.y <= 0f)
            {
                NextEpisode(-1f);
            }

            if (!tempcol.isGrounded)
            {
                m_allGrounded = false;
                break;
            }

            else
            {
                tempcol.GetGroundHit(out m_out);

                if (m_out.collider.CompareTag("DeadZone"))
                {
                    NextEpisode(-1f);
                }
            }
        }
    }
        private void NextEpisode(float _reward)
    {
        SetReward(_reward);
        EndEpisode();
    }

    private void InfiniteRewardCheck()
    {
        if (Mathf.Abs(m_currentReward) > 10000)
        {
            NextEpisode(0f);
        }
    }

    private void FixedUpdate()
    {
        m_steps++;

        m_allGrounded = true;

        InfiniteRewardCheck();
        CheckGrounded();
        CheckMovement();
    }
    private void Update()
    {
        Debug.Log("Dot product (agent forward,dirToTarget) = "+Vector3.Dot(transform.forward, m_dirToTarget)); 
    }
}
