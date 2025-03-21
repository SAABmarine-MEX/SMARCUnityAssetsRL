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
    private BrovPhysicsPrior brovPhysics;
    Vector3 inputForce = Vector3.zero;
    Vector3 inputTorque = Vector3.zero;
    
    // DRL STUFF
    private bool isHeuristic = true; // if using heuristic the forces does not need to be scaled while the rl output needs scaling 
    /*
    // For gate
    private List<Vector3> gatePositions = new List<Vector3>();
    private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero };
    private int iNextGate = 1;
     */
    // For continous rewards
    private Vector3 prevPos;
    Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
    Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);
    // DRL training parameters
    private float gamma = 0.99f;
    private float epsilon = 0.2f;
    private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f; // NOTE: lambda3=-10 in report

    public override void Initialize()
    {
        print("Init");
        brovPhysics = GetComponent<BrovPhysicsPrior>();
        prevPos = brovPhysics.GetLocalPos();
        /*
        // Get gate positions
         GameObject gates = GameObject.Find("Gates");
           if (gates != null)
           {
               // Iterate over direct children of Gates. They are already sorted from Unity scene, i.e first child is first gate, second is second etc.
               foreach (Transform child in gates.transform)
               {
                   Debug.Log("Found child: " + child.gameObject.name);
                   Debug.Log("Child's position: " + child.localPosition);
                   gatePositions.Add(child.localPosition);
                   // You can also access the child object:
                   GameObject childObject = child.gameObject;
               }
               Debug.Log("tot n:" + gatePositions.Count);
           }
           else
           {
               Debug.LogError("Gates object not found!");
           }
         */
    }

    public override void OnEpisodeBegin()
    {
        // Agent's starting state for the track
        // TODO: later, make the starting positions more random
        Vector3 localPosition = new Vector3(0f, 0f, 0f);
        Quaternion localRotation = Quaternion.Euler(0, 0, 0);
        //brovPhysics.SetZeroVels();
        //brovPhysics.SetPosAndRot(localPosition, localRotation);
		
        /*
        // Reset next gate positions
        next2Gates[0] = gatePositions[0];
        next2Gates[1] = gatePositions[1];
        iNextGate = 1;
        */
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        /* Sensor/perception input for the agent */
        // State
        sensor.AddObservation(brovPhysics.GetLocalPosNED()); // 3x1
        sensor.AddObservation(brovPhysics.GetLocalRotEulerNED()); // 3x1
        sensor.AddObservation(brovPhysics.GetVelocity()); // 6x1
		
        /* not needed for residual modelling
        // Relative position to next gate
        Vector3 relVec2Gate1 = next2Gates[0] - brovPhysics.GetLocalPos();
        sensor.AddObservation(relVec2Gate1); // Relative vector to next gate
        //Vector3 relVec2Gate2 = next2Gates[1] - brovPhysics.GetLocalPos();
        //sensor.AddObservation(relVec2Gate2); // Relative vector to second next gate
		
        // Previous action
        sensor.AddObservation(prevActions); // TODO: maybe change this to be ActionSegment data type?
        */
    }
    // What actions the agent can preform
    public override void OnActionReceived(ActionBuffers actions)
    {
        ActionSegment<float> actionsSeg = actions.ContinuousActions;
        currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]);

        if (!isHeuristic) // If action from rl, it needs to be scaled from [-1, 1] to [minValue, maxValue] for each dof
        {
            // x,y,z noted with irl coord sys
            /*
            float forceYNorm = actionsSeg[0];
            float forceZNorm = actionsSeg[1];
            float forceXNorm = actionsSeg[2];

            float torquePitchNorm = actionsSeg[3];
            float torqueYawNorm = actionsSeg[4];
            float torqueRollNorm = actionsSeg[5];
            */

            int numActions = actionsSeg.Length;
            Vector2[] ranges = new Vector2[numActions];
            // min max ranges for each dof
            // TODO: make these into variables since they are used in the heuristic as well
            ranges[0] = new Vector2(-85f, 85f); // y
            ranges[1] = new Vector2(-122f, 122f); // z
            ranges[2] = new Vector2(-85, 85); // x
            ranges[3] = new Vector2(-14f, 14f); // pitch
            ranges[4] = new Vector2(-14f, 14f); // yaw
            ranges[5] = new Vector2(-14f, 14f); // roll

            // Scale each continuous action using its specific range.
            for (int i = 0; i < numActions; i++)
            {
                actionsSeg[i] = ((actionsSeg[i] + 1f) / 2f) * (ranges[i].y - ranges[i].x) + ranges[i].x;
            }
        }
		
        inputForce  = new Vector3(actionsSeg[0], actionsSeg[1], actionsSeg[2]);
        inputTorque = new Vector3(actionsSeg[3], actionsSeg[4], actionsSeg[5]);

        brovPhysics.SetInput(inputForce, inputTorque);
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (!isHeuristic) { isHeuristic = true; } 
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;    
        // Teleop
        if (Input.GetKey(KeyCode.W))
        {
            inputForce[2] += 85;
        }

        if (Input.GetKey(KeyCode.A))
        {
            inputForce[0] -= 85;
        }

        if (Input.GetKey(KeyCode.S))
        {
            inputForce[2] -= 85;
        }

        if (Input.GetKey(KeyCode.D))
        {
            inputForce[0] += 85;
        }

        if (Input.GetKey(KeyCode.Space))
        {
            inputForce[1] += 122;
        }

        if (Input.GetKey(KeyCode.LeftShift))
        {
            inputForce[1] -= 122;
        }

        if (Input.GetKey(KeyCode.Q))
        {
            inputTorque[1] -= 14;
        }

        if (Input.GetKey(KeyCode.E))
        {
            inputTorque[1] += 14;
        }

        if (Input.GetKey(KeyCode.X))
        {
            inputTorque[0] += 14;
        }

        if (Input.GetKey(KeyCode.C))
        {
            inputTorque[2] += 14;
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
