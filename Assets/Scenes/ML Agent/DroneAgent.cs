using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using VehicleComponents.Actuators;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;

using DefaultNamespace; // ResetArticulationBody() extension


using Force;
using Unity.Mathematics;
using System;
using System.Collections.Generic;
using MathNet;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Unity.Robotics.Core; //Clock
using Unity.Robotics.ROSTCPConnector;
using StdMessages = RosMessageTypes.Std;

using Random = UnityEngine.Random;

public class DroneAgent : Agent
{
    [Header("Basics")] 
    public GameObject BaseLink;
    public Transform goal;             // Goal transform for navigation

    public DroneController.DroneController droneController;

    double massQuadrotor;
    
    public float targetHeight = 7f;  
    public float maxSpeed = 6f;     

    private ArticulationBody baseLinkDroneAB;
    private ArticulationBody[] ABparts;
    private Rigidbody[] RBparts;

    private Vector3 previousVelocity;

    private int immovableStage = 2;

    public override void Initialize()
    {
        if (goal == null)
        {
            Debug.LogWarning("Target or Goal not set for DroneAgent. Disabling.");
            enabled = false;
        }

         // Check if BaseLink exists and has an ArticulationBody component
        if (BaseLink == null)
        {
            Debug.LogError("BaseLink or ArticulationBody is missing!");
            enabled = false;
            return;
        }

        baseLinkDroneAB = BaseLink.GetComponent<ArticulationBody>();
        ABparts = BaseLink.GetComponentsInChildren<ArticulationBody>();
        RBparts = BaseLink.GetComponentsInChildren<Rigidbody>();

        massQuadrotor = baseLinkDroneAB.mass; 
    }
    

    public override void CollectObservations(VectorSensor sensor)
    {   
        var position = BaseLink.transform.position;
        sensor.AddObservation(position.x);
        sensor.AddObservation(position.y);
        sensor.AddObservation(position.z);

        var velocity = baseLinkDroneAB.linearVelocity;
        sensor.AddObservation(velocity.x);
        sensor.AddObservation(velocity.y);
        sensor.AddObservation(velocity.z);

        var acceleration = (velocity - previousVelocity) / Time.fixedDeltaTime;
        previousVelocity = velocity; // Store for next frame
        sensor.AddObservation(acceleration.x);
        sensor.AddObservation(acceleration.y);
        sensor.AddObservation(acceleration.z);

        sensor.AddObservation(goal.position.x);
        sensor.AddObservation(goal.position.y);
        sensor.AddObservation(goal.position.z);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float velX = actionBuffers.ContinuousActions[0];
        float velY = actionBuffers.ContinuousActions[1];
        float velZ = actionBuffers.ContinuousActions[2];

        float accelX = actionBuffers.ContinuousActions[3];
        float accelY = actionBuffers.ContinuousActions[4];
        float accelZ = actionBuffers.ContinuousActions[5];

        if (droneController != null)
        {
            droneController.TargetVelocity = new Vector3(velX, velY, velZ) * maxSpeed;
            droneController.TargetAccel = new Vector3(accelX, accelY, accelZ);
        }

        float distanceToGoal = Vector3.Distance(BaseLink.transform.position, goal.position);
        float velocityPenalty = -Vector3.Magnitude(baseLinkDroneAB.linearVelocity) * 0.1f;

        var reward = -distanceToGoal + velocityPenalty; 
        SetReward(reward);
        if (distanceToGoal < 1f)
        {
            SetReward(2f);
            EndEpisode();
        }

        if (distanceToGoal > 15f)
        {
            SetReward(-1f);
            EndEpisode();
        }
    }

    public void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.O))
        {
            ResetAgent();
        }

        switch (immovableStage)
        {
            case 0:
                immovableStage = 1;
                break;
            case 1:
                baseLinkDroneAB.immovable = false;
                immovableStage = 2;
                break;
            case 2:
                break;
        }
    }

    private void ResetAgent()
    {
        baseLinkDroneAB.transform.position = new Vector3(goal.position.x + 5f, targetHeight, goal.position.z + 5f);
        baseLinkDroneAB.linearVelocity = Vector3.zero;
        baseLinkDroneAB.angularVelocity = Vector3.zero;

        foreach (var ab in ABparts)
        {
            ab.linearVelocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;
            ab.ResetArticulationBody();
        }

        foreach (var rb in RBparts)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        baseLinkDroneAB.immovable = true;
        immovableStage = 0;

        previousVelocity = Vector3.zero;
        enabled = true;
    }

    public override void OnEpisodeBegin()
    {
        ResetAgent();
    }
}
