using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine.UIElements;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using VehicleComponents.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using DefaultNamespace;

public class BrovAgentPrior : Agent
{
    // For interacting with the Brov
    private BrovPhysics brovPhysics;
    Vector3 inputForce = Vector3.zero;
    Vector3 inputTorque = Vector3.zero;
    
    // if using heuristic the forces does not need to be scaled while the rl output needs scaling
    private bool isHeuristic = false;
    
    public override void Initialize()
    {
        print("Init");
        brovPhysics = GetComponent<BrovPhysics>();
        
        // Start position
        Vector3 localPosition = new Vector3(0f, -3f, 0f);
        Quaternion localRotation = Quaternion.Euler(0, 0, 0);
        //brovPhysics.SetPosAndRot(localPosition, localRotation);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        /* Sensor/perception, input for the agent */
        // State
        sensor.AddObservation(brovPhysics.GetLocalPosNED()); // 3x1
        sensor.AddObservation(brovPhysics.GetLocalRotEulerNED()); // 3x1
        sensor.AddObservation(brovPhysics.GetVelocity()); // 6x1
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        /*
         Input: actions, output from the model or heuristic action
         6x1 NED frame, [-1, 1]
         
         Output: forces and torques that will act on the brov trough BrovPhysics
         
         For residual testing in python, isHeuristic will be false
         */
        ActionSegment<float> actionsSeg = actions.ContinuousActions;
        if (!isHeuristic) // If action from rl, it needs to be scaled from [-1, 1] to [minValue, maxValue] for each dof
        {
            print("SCALE!!");
            int numActions = actionsSeg.Length;
            Vector2[] ranges = new Vector2[numActions];
            // x,y,z noted with NED coord sys
            // TODO: make these into variables since they are used in the heuristic as well
            // min max ranges for each dof
            // TODO: need to double chech that these were the max and min forces for each dof from brov datasheet or where every it was found
            ranges[0] = new Vector2(-85, 85); // x
            ranges[1] = new Vector2(-85f, 85f); // y
            ranges[2] = new Vector2(-122f, 122f); // z
            ranges[3] = new Vector2(-14f, 14f); // roll
            ranges[4] = new Vector2(-14f, 14f); // pitch
            ranges[5] = new Vector2(-14f, 14f); // yaw

            // Scale each continuous action using its specific range.
            for (int i = 0; i < numActions; i++)
            {
                actionsSeg[i] = ((actionsSeg[i] + 1f) / 2f) * (ranges[i].y - ranges[i].x) + ranges[i].x;
            }
        }
        inputForce  = new Vector3(actionsSeg[0], actionsSeg[1], actionsSeg[2]);
        inputTorque = new Vector3(actionsSeg[3], actionsSeg[4], actionsSeg[5]);
        
        brovPhysics.SetInputNED(inputForce, inputTorque);
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (!isHeuristic) { isHeuristic = true; } // If in this method, then heuristic is true
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions; 
        
        // Teleop
        if (Input.GetKey(KeyCode.W))
        {
            inputForce[0] += 85;
        }
        if (Input.GetKey(KeyCode.A))
        {
            inputForce[1] -= 85;
        }
        if (Input.GetKey(KeyCode.S))
        {
            inputForce[0] -= 85;
        }
        if (Input.GetKey(KeyCode.D))
        {
            inputForce[1] += 85;
        }
        if (Input.GetKey(KeyCode.Space))
        {
            inputForce[2] += 122;
        }
        if (Input.GetKey(KeyCode.LeftShift))
        {
            inputForce[2] -= 122;
        }
        if (Input.GetKey(KeyCode.Q))
        {
            inputTorque[2] -= 14;
        }
        if (Input.GetKey(KeyCode.E))
        {
            inputTorque[2] += 14;
        }
        if (Input.GetKey(KeyCode.X))
        {
            inputTorque[1] += 14;
        }
        if (Input.GetKey(KeyCode.C))
        {
            inputTorque[0] += 14;
        }
        
        // Forces
        continuousActions[0] = inputForce[0];
        continuousActions[1] = inputForce[1];
        continuousActions[2] = inputForce[2];
        // Torques
        continuousActions[3] = inputTorque[0];
        continuousActions[4] = inputTorque[1];
        continuousActions[5] = inputTorque[2];
    }
    void FixedUpdate()
    {
        // Reset input forces every fixed update
        inputForce = Vector3.zero;
        inputTorque = Vector3.zero;
    }
}
