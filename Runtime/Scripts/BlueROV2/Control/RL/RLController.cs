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

/*
TODO:
- make CheckpointsManager into a separete script
- make random position to gates
*/

namespace DefaultNamespace.BlueROV2.Control
{
    public class RLController : Agent
    {
        // Necessary components
        public BrovDynamics dynamics;
        private GameObject map;
        public TeleopController teleopController;
        
        
        // RL stuff
        // Gates
        private GameObject gates;
        // List to hold all checkpoint GameObjects
        private List<GameObject> checkpoints = new List<GameObject>();
        // List to hold the two next gates, in NED
        private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero };
        // Index of next gate. First gate has index 0
        private int iNextGate = 0;
        // Distances to next gate
        private Vector3 d_prev;
        private Vector3 d_curr;
        
        Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
        Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);
        private Vector3 startPos;
        private Quaternion startQ;
        private Vector3 prevPos;
        private Vector3 currPos;
        
        // Reward weights
        private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f; // lambdas from https://www.nature.com/articles/s41586-023-06419-4
        private float w_progress = 80f, w_allign = 0.1f, w_smoothness = 0.2f; // w_progress = 80 so that when brov drives with 50% forward, it will represent +~1 TODO: would be nice to have these relative each other for better interpetation of the weights

        
        [Header("Random spawn options")]
        //public Vector3 spawnAreaCenter = Vector3.zero;
        //public Vector3 spawnAreaSize = new Vector3(2f, -2f, 9f); // Safe size of tank, unity coordinate frame
        //public int collisionMask = 0; // Default to everything
        public float checkRadius = 0.2f; // How close it can be to another object
        public int maxAttempts = 10; // Max tries before giving up
        //public bool randomizeRotation = true; // Toggle if you want random rotation
        

        public void Setup(GameObject mapframe) 
        {
            map = mapframe;
        }
        
        void Start()
        {
            // Check if components exists
            if (dynamics == null) { Debug.LogError("dynamics not set"); }
            if (map == null) { Debug.LogError("map not set"); }
            
            // Get init position
            startQ = dynamics.GetQuaternionNED();
            startPos = dynamics.GetPosNED();
            prevPos = startPos;
            currPos = prevPos;
            
            // Find the gates
            Transform current = transform.parent;
            while (current != null)
            {
                foreach (Transform sibling in current)
                {
                    if (sibling.name == "Checkpoints")
                    {
                        gates = sibling.gameObject;
                        break;
                    }
                }
                current = current.parent;
            }
            if (gates == null)
            {
                Debug.LogError("'Checkpoints' GameObject not found in parent hierarchy.");
            }
            else // Add the checkpoints to the checkpoints list
            {
                // ASSUMPTION: Gates are sorted in the desired track order in the Unity scene
                for (int i = 0; i < gates.transform.childCount; i++)
                {
                    Transform child = gates.transform.GetChild(i);
                    GameObject childGameObject = child.gameObject;
                    childGameObject.GetComponent<CheckpointSingle>().checkpointIndex = i;
                    checkpoints.Add(childGameObject);
                }
            }
        }
        
        public override void OnEpisodeBegin()
        {
            // Make brov be still
            dynamics.SetInputTauNED(new float[] {0f, 0f, 0f, 0f, 0f, 0f});
            dynamics.SetZeroVels();
            
            // Reset the Brov to its starting position
            RandomizeStartPosition();
            //FindClosestWaypoint();
            
            // Reset next gate positions to the first two gates in the track
            //if (iNextGate+1 == gatePositions.Count) { next2Gates[1] = gatePositions[0]; }
            //else { next2Gates[1] = gatePositions[iNextGate+1]; }
            iNextGate = 0;
            
            Vector3 checkpoint1Pos = map.transform.InverseTransformPoint(checkpoints[0].transform.position);
            var gatePos1NED = checkpoint1Pos.To<NED>().ToUnityVec3();
            
            Vector3 checkpoint2Pos = map.transform.InverseTransformPoint(checkpoints[1].transform.position);
            var gatePos2NED = checkpoint1Pos.To<NED>().ToUnityVec3();
            
            next2Gates[0] = gatePos1NED;
            next2Gates[1] = gatePos2NED;
        }

        public void RandomizeStartPosition()
        {
            int attempts = 0;
            bool foundPosition = false;
            
            Vector3 startUnity = NED.ConvertToRUF(startPos);

            while (attempts < maxAttempts && !foundPosition)
            {
                Vector3 randomOffset = new Vector3(
                    UnityEngine.Random.Range(-0.5f, 0.5f),
                    UnityEngine.Random.Range(-0.5f, 0.5f),
                    UnityEngine.Random.Range(-0.5f, 0.5f)
                );
                Vector3 randomPosition = startUnity + randomOffset;
                // Convert local position relative to the map object to world position
                Vector3 worldPosition = map.transform.TransformPoint(randomPosition);
                
                int colMask = LayerMask.GetMask("Default");
                if (!UnityEngine.Physics.CheckSphere(worldPosition, checkRadius, colMask))
                { 
                    Quaternion randomRot = RandomRotation();
                    dynamics.SetPose(randomPosition, randomRot);
                    foundPosition = true;
                }
                attempts++;
            }

            if (!foundPosition)
            {
                Debug.LogWarning("Could not find a free spot to spawn after " + maxAttempts + " attempts.");
            }
        }
        
        /*
        public void FindClosestWaypoint()
        {
            float closestDistance = Mathf.Infinity;
            for (int i = 0; i < gatePositions.Count; i++)
            {
                float distance = Vector3.Distance(dynamics.GetPosNED(), gatePositions[i]);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    iNextGate = i;
                }
            }
        }
        */
        
        private Quaternion RandomRotation()
        {
            // Random rotation around each axis, small angles
            Vector3 randomEuler = new Vector3(
                UnityEngine.Random.Range(-10f, 10f),  // Roll (x)
                UnityEngine.Random.Range(-10f, 10f),  // Pitch (y)
                UnityEngine.Random.Range(-10f, 10f)   // Yaw (z) // before +/- 45DEG
            );
            
            // Convert to Quaternion
            Quaternion randomRotation = Quaternion.Euler(randomEuler);
            
            // Apply relative to current rotation
            Quaternion newRot = startQ * randomRotation;
            Quaternion qRUF = NED.ConvertToRUF(newRot);

            return qRUF;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            /*
            * This method adds observations to the sensor of the agent.
            * The observations are used as input to the neural network.
            */
            // 1. State
            sensor.AddObservation(dynamics.GetQuaternionNED()); // 1x4
            sensor.AddObservation(dynamics.GetVelsNED()); // 1x6
            
            // 2. Relative position to next gate
            Vector3 relVecToGateInput = next2Gates[0] - dynamics.GetPosNED(); // 1x3
            sensor.AddObservation(relVecToGateInput);
            
            // 3. Previous action
            sensor.AddObservation(prevActions); // 1x6
        }
        

        public override void OnActionReceived(ActionBuffers actions)
        {
            ActionSegment<float> actionsSeg = actions.ContinuousActions;
            currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]*0.5f); // scale down for safety
            
            float r = CalculateReward();
            AddReward(r);
            
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

        private void FixedUpdate()
        {
            if (dynamics.GetHeight() >= 0)
            {
                AddReward(-1.0f);
                EndEpisode();
            }
        }
        /*
        void OnDrawGizmos()
        {
            foreach (Transform child in gates.transform)
            {
                checkpoints.Add(child.gameObject);
                Gizmos.DrawSphere(child.transform.position, 0.1f); // Radius 0.3 for sphere
                
                // Draw the direction of the checkpoint (arrow)
                Gizmos.color = Color.red; // Color for the direction arrow  
                Gizmos.DrawRay(child.transform.position, child.transform.forward * 1.0f); // Draw an arrow
                
            }
            Gizmos.color = Color.green;
            Gizmos.DrawRay(dynamics.mainBody.transform.position, dynamics.mainBody.transform.forward * 1.0f);
        }
        */

        private float CalculateReward()
        {
            // 1) Progression reward
            float d_prev = Vector3.Distance(next2Gates[0], prevPos);
            currPos = dynamics.GetPosNED();
            float d_curr = Vector3.Distance(next2Gates[0], currPos);
            float r_prog = w_progress * (d_prev - d_curr);
            //print("R PROG: " + r_prog);
            
            
            // 2) Facing gate / allign with target reward
            var r = checkpoints[iNextGate].transform.forward;
            var d = dynamics.GetForwardUnitVec();
            //float r_allign = w_allign * ((Vector3.Dot(r, d) + 1) * 0.5f); // w_allign * [0.0, 1.0]
            float r_allign = w_allign * ((Vector3.Dot(r, d))); // w_allign * [-1.0, 1.0]
            //print("R ALLIGN: " + r_allign);
            
            
            // 3) Smooth action reward
            //Vector<float> actionDiff = currActions - prevActions;
            float squaredDifference = 0.0f;
            for (int i = 0; i < currActions.Count; i++)
            {
                float diff = currActions[i] - prevActions[i];
                squaredDifference += diff * diff; // max value: 1 since biggest possible diff is 1 
            }
            //float r_smooth = lambda5 * squaredDifference;
            float r_smooth = w_smoothness * squaredDifference;
            
            /*
            // Different types of smooth reward function 
            float diffSum = 0f;
            for (int i = 0; i < currActions.Count; i++) {
                diffSum += Mathf.Abs(currActions[i] - prevActions[i]);
            }
            float r_cmd2 = w_smoothness * (-diffSum / (2 * currActions.Count));
            //print("SMOOOOTHH GPT FIRST");
            //Vector<float> diff = currActions - prevActions;
            //Debug.Log(diff[0] + "     " + diff[1] + "     " + diff[2] + "     " + diff[3] + "           " + diff[4] + "     " + diff[5]);
            //Debug.Log(r_cmd2);
            
            float maxL2 = Mathf.Sqrt(currActions.Count); // e.g., sqrt(6)
            float smoothness = 1f - Mathf.Clamp((float) actionDiff.L2Norm() / maxL2, 0f, 1f);
            //print("SMOOOOOOOTH l2");
            float r_smooth = w_smoothness * smoothness;
            //print("R SMOOTH: " + smoothness);
            
            
            float magnitude = Mathf.Pow(actionDiffNorm, 2);
            float r_cmd = lambda5*magnitude;
            
            float sqDiff = 0f;
            for (int i = 0; i < 6; i++)
            {
                float dd = currActions[i] - prevActions[i];
                sqDiff += dd * dd;
            }
            float rSmooth = Mathf.Exp(-1 * sqDiff);
            //print("SMOOOOOTH: "+ rSmooth);
            */
		
            // Sum tot reward
            //float r_t = r_prog + r_allign + r_smooth;
            // Do without r_allign now since we know yaw is trash in real,
            // so try to create a policy that dont yaw as much
            float r_t = r_prog + r_smooth;

            return r_t;
        }
    
        private void OnTriggerEnter(Collider other)
        {
            CheckpointSingle cpData = other.GetComponent<CheckpointSingle>();
            if (cpData != null)
            {
                if (cpData.checkpointIndex == iNextGate)
                {
                    AddReward(5f);
                    if ((iNextGate + 1) == checkpoints.Count)
                    {
                        iNextGate = 0;
                        EpisodeInterrupted();
                        return;
                    }
                    iNextGate = (iNextGate + 1) % checkpoints.Count;
                    
                    Vector3 nextGate = map.transform.InverseTransformPoint(checkpoints[iNextGate].transform.position);
                    nextGate = nextGate.To<NED>().ToUnityVec3();
                    next2Gates[1] = nextGate; 
                    next2Gates[0] = next2Gates[1];
                }else{
                    //Debug.Log("Wrong order!");
                    //AddReward(-1f);
                }
            }
            
            if (other.gameObject.tag == "Wall")
            {
                AddReward(-1f);
                EndEpisode();
            }
        }
    }
}