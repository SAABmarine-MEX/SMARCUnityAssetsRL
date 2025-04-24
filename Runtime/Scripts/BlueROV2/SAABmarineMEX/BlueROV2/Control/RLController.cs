using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Physics;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;
using System.Collections.Generic;
using System;


namespace DefaultNamespace.BlueROV2.Control
{
    public class RLController : Agent
    {
        public BoxCollider box; // for OnTrigger to work
        public BrovDynamics dynamics;
        private GameObject map;
        public TeleopController teleopController;
        
        
        // RL stuff
        // For gates
        private List<Vector3> gatePositions = new List<Vector3>(); //TODO:
        private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero };
        private int iNextGate = 0;
        
        // For continous rewards
        Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
        Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);
        private Vector3 prevPos;
        private Vector3 currPos;
        
        // RL training parameters
        private float gamma = 0.99f;
        private float epsilon = 0.2f;
        private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f; // NOTE: lambda3=-10 in report
        
        // Control input variables
        private Quaternion qInput;
        private Vector<float> velsInput;
        private Vector3 relVecToGateInput;
        // prevActions also used as control input but declared above
        

        public void Setup(GameObject mapframe)
        {
            map = mapframe;
        }
        void Start()
        {
            print("RL START");
            if (dynamics == null)
            {
                Debug.LogError("dynamics not set");
            }
            map = GameObject.Find("map"); // Map frame as in ros
            
            // Init position
            prevPos = dynamics.GetPosNED();
            currPos = prevPos;
            
            // Get checkpoints positions
            GameObject gates = GameObject.Find("Checkpoints");
            print("Gates:");
            if (gates != null)
            {
                // Assumption: Gates are sorted in the desired track order in the Unity scene, i.e. first child is first gate etc.
                foreach (Transform child in gates.transform)
                {
                    Vector3 localPosition = map.transform.InverseTransformPoint(child.transform.position);
                    //print("map frame: x" + localPosition.To<NED>().ToDense()[0]);
                    //print("map frame: y" + localPosition.To<NED>().ToDense()[1]);
                    //print("map frame: z" + localPosition.To<NED>().ToDense()[2]);
                    
                    var gatePosTemp = localPosition.To<NED>().ToDense();
                    Vector3 gatePos = new Vector3((float)gatePosTemp[0], (float)gatePosTemp[1], (float)gatePosTemp[2]);
                    gatePositions.Add(gatePos);
                    print(gatePos[0] + "    " + gatePos[1]  + "    " + gatePos[2]);
                }
            }
            else
            {
                Debug.LogError("Gates not set");
            }
        }

        public override void OnEpisodeBegin()
        {
            dynamics.SetInputTauNED(new float[] {0f, 0f, 0f, 0f, 0f, 0f});
            dynamics.SetZeroVels();
            // Reset the Brov to its starting position TODO: later, make the starting position more random
            Vector3 localStartPos = new Vector3(-1.3f, -2.0f, 1.0f); // TODO: make it relative to the same origin as it will be irl
            Quaternion localStartRot = Quaternion.Euler(0f, 0f, 0f); // TODO: would like this to be relative map
            dynamics.SetPose(localStartPos, localStartRot);
            
            // Reset next gate positions to the first two gates TODO: only have nex gate
            next2Gates[0] = gatePositions[0];
            next2Gates[1] = gatePositions[1];
            iNextGate = 0;
            
            
            // Init control input variables
            // 1. State
            qInput = dynamics.GetQuaternionNED(); // 1x4
            velsInput = dynamics.GetVelsNED(); // 1x6
            
            // 2. Relative position to next gate
            relVecToGateInput = next2Gates[0] - dynamics.GetPosNED();
            
            // 3. Previous action, is already initilized
        }

        public void SetDynamics(BrovDynamics dyns)
        {
            dynamics = dyns;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            /*
            * This method adds observations to the sensor of the agent. 
            * The observations are used as input to the neural network.
            */
            // 1. State
            qInput = dynamics.GetQuaternionNED(); // 1x4
            velsInput = dynamics.GetVelsNED(); // 1x6
            sensor.AddObservation(qInput); 
            sensor.AddObservation(velsInput);
            
            // 2. Relative position to next gate
            relVecToGateInput = next2Gates[0] - dynamics.GetPosNED(); // 1x3
            sensor.AddObservation(relVecToGateInput); 
            
            // 3. Previous action
            sensor.AddObservation(prevActions);
        }
        
        // Methods used for ros
        public List<float> GetControlInput()
        {
            /*
             * This method returns the input to the neural network model.
             * Simply returning the observations from the CollectObservations method.
             */
            List<float> modelInput = new List<float>();

            // 1. State
            List<float> quaternionList = new List<float> { qInput.x, qInput.y, qInput.z, qInput.w };
            modelInput.AddRange(quaternionList);
            modelInput.AddRange(velsInput);

            // 2. Relative position to next gate
            // TODO: make next2gates into float array so this code can be simplified
            float[] floatArray2 = { relVecToGateInput.x, relVecToGateInput.y, relVecToGateInput.z };
            modelInput.AddRange(floatArray2);

            // 3. Previous action
            modelInput.AddRange(prevActions);
            
            return modelInput;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            //print("GET ACTIOOOOOONS");
            ActionSegment<float> actionsSeg = actions.ContinuousActions;
            currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]*0.5f); // scale down
            //print(currActions);
            
            ContinousRewards();
            
            prevPos = currPos;
            prevActions = currActions;
        }
        
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // NOTE: When running inference from ros, set Behavior type to Inference only in the Behavior Parameters component
            // so this Heuristic doesn't interfere. Could probably solve it in a smart more dynamic way but works for now
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            Vector<float> teleopInput = teleopController.GetTeleopInput();
            for (int i = 0; i < continuousActions.Length; i++)
            {
                continuousActions[i] = teleopInput[i];
            }
        }
        private Vector<float> ScaleActions(Vector<float> actions)
        {
            Vector<float> scaledActions = Vector<float>.Build.Dense(actions.Count);
            for (int i = 0; i < actions.Count; i++)
            {
                // Scale from [-1, 1] to [1100, 1900]
                scaledActions[i] = (actions[i] + 1f) * 0.5f * (1900f - 1100f) + 1100f;
            }
            return scaledActions;
        }

        public Vector<float> GetScaledActions()
        {
            return ScaleActions(currActions);
        }

        private float ComputeReward()
        {
            // We manually ensure the rewards never exceed the -1 : 1 range during an episode.
            // This is for stability in the neural networks.
            // We dont use normalization in the network inputs, and do it manually ourselves.
            // Doing it ourselves, we dont have to "learn" what the possible range of values is.
            // IF you do this manually, make sure to turn off normalization in the learning config file.

            // Progression reward
            Math.Clamp((maxDistance - current) / maxDistance, 0, 1);
        }
        
        private void ContinousRewards()
        {
            // r_progression
            float d_prev = Vector3.Distance(next2Gates[0], prevPos);
            currPos = dynamics.GetPosNED();
            float d_curr = Vector3.Distance(next2Gates[0], currPos);
            float r_prog = 6*lambda1 * (d_prev - d_curr);

            // r_perception
            Vector3 directionToGate = (NED.ConvertToRUF(next2Gates[0]) - NED.ConvertToRUF(currPos)).normalized;
            //Debug.DrawRay(brovPhysics.mainBody.transform.position, directionToGate * 2f, Color.red);
            //Debug.DrawRay(brovPhysics.mainBody.transform.position, brovPhysics.GetForwardUnitVec() * 2f, Color.blue);
            //directionToGate.y = 0;
            float angleToGate = Vector3.SignedAngle(dynamics.GetForwardUnitVec(), directionToGate, Vector3.up); // yaw angle to gate
            // TODO: this formula doesnt make sense??? from the report
		    float part = lambda3 * Mathf.Pow(angleToGate, 4); // NOTE: pow of 2 instead of 4 as in the report
            //float r_perc = lambda2 * Mathf.Exp(part);
            float r_perc = 0;

            Vector<float> actionDiff = currActions - prevActions;

            float actionDiffNorm = (float) actionDiff.L2Norm();
            float magnitude = Mathf.Pow(actionDiffNorm, 2);
            float r_cmd = lambda5*magnitude;
            //print("CMD REWARD: " + magnitude);
            
            if (dynamics.GetHeight() >= 0)
            {
                //Debug.LogError("AAAAJ TAK");
                AddReward(-5.0f);
                EndEpisode();
                
            }
		
        // Sum tot reward
        float r_t = r_prog + r_perc + r_cmd;        
		AddReward(r_t);
        //print("TOT REWARD: " + r_t);
    }
    
    private void OnTriggerEnter(Collider other)
    {
        //print("TRIGGER ENTER");
        CheckpointSingle cpData = other.GetComponent<CheckpointSingle>();
        if (cpData != null)
        {
            if (cpData.checkpointIndex == iNextGate)
            {
                Debug.Log("RÄTT ORDNING");
                AddReward(5f);
                iNextGate = (iNextGate + 1) % gatePositions.Count; 				
                next2Gates[0] = next2Gates[1];
                next2Gates[1] = gatePositions[iNextGate]; 
            }else{
                // TODO: fix so that it doesnt give this multiple times when passing through
                // Wrong order!
                //Debug.Log("FEL ORDNING");
                //AddReward(-1f); // TODO: den vart bättre med denna men eftersom den är skum så borde det inte bli så??
            }
        }
        if (other.gameObject.tag == "Wall")
        {
            //print("AJ VÄGG");
            AddReward(-5f);
            EndEpisode();
        }
    }

    }
}