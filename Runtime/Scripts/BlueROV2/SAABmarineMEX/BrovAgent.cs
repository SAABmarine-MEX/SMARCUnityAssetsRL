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

public class BrovAgent : Agent
{
    // For interacting with the Brov
    private BrovPhysics brovPhysics;
    Vector3 inputForce = Vector3.zero;
    Vector3 inputTorque = Vector3.zero;
    private TeleopController teleController;

    // RL stuff
    // If using heuristic the forces does not need to be scaled while the rl output needs scaling
    private bool isHeuristic = false;
    // For gate
    private List<Vector3> gatePositions = new List<Vector3>();
    private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero };
    private int iNextGate = 0;
    // For continous rewards
    private Vector3 prevPos;
    private Vector3 currPos;
    Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
    Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);
    // RL training parameters
    private float gamma = 0.99f;
    private float epsilon = 0.2f;
    private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f; // NOTE: lambda3=-10 in report

    public override void Initialize()
    {
        Debug.Log("Init agent: " + gameObject.name);
        brovPhysics = GetComponentInParent<BrovPhysics>();
        teleController = GetComponentInParent<TeleopController>();
        prevPos = brovPhysics.GetLocalPosNED();
        currPos = prevPos;

        // Get gate positions
        GameObject gates = GameObject.Find("Gates");
        if (gates != null)
        {
            // Assumption: Gates are sorted in the desired track order in the Unity scene, i.e. first child is first gate etc.
            foreach (Transform child in gates.transform)
            {
                var gatePosTemp = child.localPosition.To<NED>().ToDense(); // Convert to NED frame instead of Unity standard RUF frame
                Vector3 gatePos = new Vector3((float)gatePosTemp[0], (float)gatePosTemp[1], (float)gatePosTemp[2]);
                gatePositions.Add(gatePos);
               }
        }
        else
        {
            Debug.LogError("Gates object not found!");
        }
    }

    public override void OnEpisodeBegin()
    {
        // Reset the Brov to its starting position TODO: later, make the starting position more random
        Vector3 localStartPos = new Vector3(3.5f, 0.5f, 0.4f); // TODO: make it relative to the same origin as it will be irl
        Quaternion localStartRot = Quaternion.Euler(0, 0, 0);
        brovPhysics.SetZeroVels();
        brovPhysics.SetPosAndRot(localStartPos, localStartRot);

        // Reset next gate positions to the first two gates
        next2Gates[0] = gatePositions[0];
        next2Gates[1] = gatePositions[1];
        iNextGate = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        /*
         * This method adds observations to the sensor of the agent. 
         * The observations are used as input to the neural network.
         */

        // 1. State
        //sensor.AddObservation(brovPhysics.GetLocalPos());
        sensor.AddObservation(brovPhysics.GetLocalRotEulerNED());
        sensor.AddObservation(brovPhysics.GetVelocity());

        // 2. Relative position to next gate NOTE: Now it only uses the first gate as that is how it is done in the drone racing paper
        Vector3 relVec2Gate1 = next2Gates[0] - brovPhysics.GetLocalPosNED();
        sensor.AddObservation(relVec2Gate1);
        //Vector3 relVec2Gate2 = next2Gates[1] - brovPhysics.GetLocalPos();
        //sensor.AddObservation(relVec2Gate2); // Relative vector to second next gate

        // 3. Previous action
        sensor.AddObservation(prevActions); // TODO: maybe change this to be ActionSegment data type? Does it matter?
    }

    public List<float> GetModelInput()
    {
        /*
         * This method returns the input to the neural network model.
         * Simply returning the observations from the CollectObservations method.
         */
        List<float> modelInput = new List<float>();

        // 1. State
        modelInput.AddRange(brovPhysics.GetLocalRotEulerNED());
        modelInput.AddRange(brovPhysics.GetVelocity());

        // 2. Relative position to next gate
        // TODO: make next2gates into float array so this code can be simplified
        Vector3 relVec2Gate1 = next2Gates[0] - brovPhysics.GetLocalPosNED();
        float[] floatArray2 = new float[] { relVec2Gate1.x, relVec2Gate1.y, relVec2Gate1.z };
        modelInput.AddRange(floatArray2);

        // 3. Previous action
        modelInput.AddRange(prevActions);
        return modelInput;
    }

    // What actions the agent can preform
    public override void OnActionReceived(ActionBuffers actions)
    {
        /*
         Input: actions, output from the model or heuristic action
         6x1 NED frame, [-1, 1]
         
         Output: forces and torques that will act on the brov trough BrovPhysics
         */
        /*
         * NEW VERSION
         * ActionSegment<float> actionsSeg = actions.ContinuousActions;
         * currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]);
         * float[] actionsScaled = brovPhysics.ScaleActions(currActions);
         * add actionsScaled into actionsSeg
         * add to brovPhysics.setInputNED(slicea den, fint)
         */

        ActionSegment<float> actionsSeg = actions.ContinuousActions; // TODO: will this init as all zeros? test
        currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]);

            // x,y,z noted with irl coord sys
            // TODO: make the scaling into a method
            int numActions = actionsSeg.Length;
            Vector2[] ranges = new Vector2[numActions];
            // min max ranges for each dof
            // TODO: make these into variables since they are used in the heuristic as well
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

		
        inputForce  = new Vector3(actionsSeg[0], actionsSeg[1], actionsSeg[2]);
        inputTorque = new Vector3(actionsSeg[3], actionsSeg[4], actionsSeg[5]);
        brovPhysics.SetInputNED(inputForce, inputTorque);
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        float[] teleopInput = teleopController.GetTeleopInput();
        // TODO: make below into a for-loop
        // Forces
        continuousActions[0] = teleopInput[0];
        continuousActions[1] = teleopInput[1];
        continuousActions[2] = teleopInput[2];
        // Torques
        continuousActions[3] = teleopInput[3];
        continuousActions[4] = teleopInput[4];
        continuousActions[5] = teleopInput[5];
    }
    private void ContinousRewards() // TODO: make sure it does this at each time step
    {
        // r_progression
        float d_prev = Vector3.Distance(next2Gates[0], prevPos);
        //print("d_prev " + d_prev);
        currPos = brovPhysics.GetLocalPosNED();
        float d_curr = Vector3.Distance(next2Gates[0], currPos);
        float r_prog = 4*lambda1 * (d_prev - d_curr);
        //print("PROG REWARD: " + r_prog);

        // r_perception
        Vector3 directionToGate = (NED.ConvertToRUF(currPos) - NED.ConvertToRUF(next2Gates[0])).normalized;
		//print(directionToGate);
		//print("ANGLE: " + Vector3.Angle(brovPhysics.GetForwardUnitVec(), NED.ConvertToRUF(directionToGate)));
        //float angleToGate = Mathf.Acos(Vector3.Dot(brovPhysics.GetForwardUnitVec(), directionToGate)) * Mathf.Rad2Deg; // TODO: why minsta runt 30??
        float angleToGate = Vector3.Angle(brovPhysics.GetForwardUnitVec(), NED.ConvertToRUF(directionToGate));
		//print("angle to gate " + angleToGate);
		float part = lambda3 * Mathf.Pow(angleToGate, 2); // NOTE: pow of 2 instead of 4 as in the report
        float r_perc = lambda2 * Mathf.Exp(part);
        //print("PERC REWARD: " + r_perc);
	
        // r_command
        // TODO: osäker om rätt implementerad
        /*
        print("PREV ACTION:");
        print(prevActions[0]);
        print(prevActions[1]);
        print(prevActions[2]);
        print(prevActions[3]);
        print(prevActions[4]);
        print(prevActions[5]);

        print("CURR ACTION:");
        print(currActions[0]);
        print(currActions[1]);
        print(currActions[2]);
        print(currActions[3]);
        print(currActions[4]);
        print(currActions[5]);
		*/
        Vector<float> actionDiff = currActions - prevActions;
/*
        print("diff vec: " + actionDiff[0]);
        print("diff vec: " + actionDiff[1]);
        print("diff vec: " + actionDiff[2]);
        print("diff vec: " + actionDiff[3]);
        print("diff vec: " + actionDiff[4]);
        print("diff vec: " + actionDiff[5]);
*/
        float actionDiffNorm = (float) actionDiff.L2Norm();
        float magnitude = Mathf.Pow(actionDiffNorm, 2);
        float r_cmd = lambda5*magnitude;
        //print("CMD REWARD: " + magnitude);
		
        // Sum tot reward
        float r_t = r_prog + r_perc + r_cmd;        
		AddReward(r_t);
        //print("TOT REWARD: " + r_t);
        
        prevPos = currPos;
        prevActions = currActions;
    }
    
    private void OnTriggerEnter(Collider other)
    {
        print("OnTriggerEnter");
        // Try to get the CheckpointData component from the collider.
        CheckpointSingle cpData = other.GetComponent<CheckpointSingle>();
        if (cpData != null)
        {
            //Debug.Log("CHECKPOINT INDEX: " + cpData.checkpointIndex);
            //Debug.Log("CORRECT INDEX: " + iNextGate);
            // Optionally, verify the checkpoint order.
            if (cpData.checkpointIndex == iNextGate)
            {
                Debug.Log("RÄTT ORDNING");
                //AddReward(10f);
                // Move to the next gate
                // TODO: make sure they are in order
                iNextGate = (iNextGate + 1) % gatePositions.Count; 				
                next2Gates[0] = next2Gates[1];
                next2Gates[1] = gatePositions[iNextGate]; 
            }else{
                // TODO: fix so that it doesnt give this multiple times when passing through
                // Wrong order!
                Debug.Log("FEL ORDNING");
                //AddReward(-1f); // TODO: den vart bättre med denna men eftersom den är skum så borde det inte bli så??
            }
        }
        if (other.gameObject.tag == "Wall")
        {
            print("AJ VÄGG");
            AddReward(-5f);
            EndEpisode();
        }
    }
    
    void FixedUpdate()
    {
        ContinousRewards();
        // Reset input forces every fixed update
        inputForce = Vector3.zero;
        inputTorque = Vector3.zero;
    }
}
