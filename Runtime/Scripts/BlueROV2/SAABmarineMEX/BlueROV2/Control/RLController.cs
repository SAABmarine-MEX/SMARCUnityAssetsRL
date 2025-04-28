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
        // Necessary components
        public BrovDynamics dynamics;
        private GameObject map;
        public TeleopController teleopController;
        
        
        // RL stuff
        // Gates
        private List<Vector3> gatePositions = new List<Vector3>(); //TODO: replace with Checkpoint list
        private List<GameObject> checkpoints = new List<GameObject>(); // List to hold the checkpoint GameObjects
        private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero };
        private int iNextGate = 0;
        
        // Reward
        Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
        Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);
        private Vector3 startPos;
        private Vector3 prevPos;
        private Vector3 currPos;

        private float maxDistance = 5f;
        
        // Training parameters
        private float gamma = 0.99f;
        private float epsilon = 0.2f;
        private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f; // NOTE: lambda3=-10 in report
        private float w1 = 6f, w2 = 6f, w3 = 6f, w4 = 6f;
        
        
        // Control input variables
        private Quaternion qInput;
        private Vector<float> velsInput;
        private Vector3 relVecToGateInput;
        // prevActions also used as control input but declared above
        
        public Vector3 spawnAreaCenter = Vector3.zero;
        public Vector3 spawnAreaSize = new Vector3(2f, -2f, 9f); // ENU safe size of tank
        public int collisionMask = 0; // Default to everything
        public float checkRadius = 0.2f; // How close it can be to another object
        public int maxAttempts = 10; // Max tries before giving up

        public bool randomizeRotation = true; // Toggle if you want random rotation
        

        
        private GameObject gates;

        public void Setup(GameObject mapframe)
        {
            map = mapframe;
        }
        void Start()
        {
            // List to hold the checkpoint GameObjects
            //List<GameObject> checkpoints = new List<GameObject>();
            
            print("RL START");
            if (dynamics == null)
            {
                Debug.LogError("dynamics not set");
            }
            map = GameObject.Find("map"); // Map frame as in ros
            
            // Init position
            startPos = dynamics.GetPosNED();
            print("Start pos:   " + startPos[0]+ "      " + startPos[1] + "      " + startPos[2]);
            prevPos = startPos;
            currPos = prevPos;
            
            // Get checkpoints positions
            gates = GameObject.Find("Checkpoints");
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
            
            
            Gizmos.color = Color.green; // Color for spheres

            //GameObject gates = GameObject.Find("Checkpoints");
            print("Gates:");
            if (gates != null)
            {
                // Assumption: Gates are sorted in the desired track order in the Unity scene
                foreach (Transform child in gates.transform)
                {
                    checkpoints.Add(child.gameObject);
                    print("Added checkpoint: " + child.gameObject.name);
                    
                }
            }
            else
            {
                Debug.LogError("Gates not set!!!!!!!!!!");
            }
        }
        
        

        public override void OnEpisodeBegin()
        {
            dynamics.SetInputTauNED(new float[] {0f, 0f, 0f, 0f, 0f, 0f});
            dynamics.SetZeroVels();
            // Reset the Brov to its starting position TODO: later, make the starting position more random
            Vector3 localStartPos = new Vector3(-1.3f, -2.0f, 1.0f); // TODO: make it relative to the same origin as it will be irl
            Quaternion localStartRot = Quaternion.Euler(0f, 0f, 0f); // TODO: would like this to be relative map
            //dynamics.SetPose(localStartPos, localStartRot);
            RandomizeStartPosition();
            //FindClosestWaypoint();
            
            // Reset next gate positions to the first two gates TODO: only have nex gate
            //next2Gates[0] = gatePositions[iNextGate];
            //if (iNextGate+1 == gatePositions.Count) { next2Gates[1] = gatePositions[0]; }
            //else { next2Gates[1] = gatePositions[iNextGate+1]; }
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
        
        public void RandomizePosition()
        {
            int attempts = 0;
            bool foundPosition = false;

            while (attempts < maxAttempts && !foundPosition)
            {
                Vector3 randomOffset = new Vector3(
                    UnityEngine.Random.Range(0f, spawnAreaSize.x),
                    UnityEngine.Random.Range(0f, spawnAreaSize.y),
                    UnityEngine.Random.Range(0f, spawnAreaSize.z)
                );

                Vector3 randomPosition = spawnAreaCenter + randomOffset;

                if (!UnityEngine.Physics.CheckSphere(randomPosition, checkRadius, collisionMask))
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

        public void RandomizeStartPosition()
        {
            int attempts = 0;
            bool foundPosition = false;

            while (attempts < maxAttempts && !foundPosition)
            {
                Vector3 randomOffset = new Vector3(
                    UnityEngine.Random.Range(-0.5f, 0.5f),
                    UnityEngine.Random.Range(-0.5f, 0.5f),
                    UnityEngine.Random.Range(-0.5f, 0.5f)
                );
                Vector3 startUnity = new Vector3(
                    startPos.y, // NED Y -> Unity X
                    -startPos.z, // NED Z -> Unity Y (inverted)
                    startPos.x // NED X -> Unity Z
                );
                Vector3 randomPosition = startUnity + randomOffset;
                //Vector3 randomPositionUnity = randomPosition.To<ENU>().ToUnityVec3();
                // Convert local position relative to the map object to world position
                Vector3 worldPosition = map.transform.TransformPoint(randomPosition);

                if (!UnityEngine.Physics.CheckSphere(worldPosition, checkRadius, collisionMask)) //TODO: dont know if it works
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
        
        private Quaternion RandomRotation()
        {
            // TODO: make it relative start orientation. baslically +/- start orientation
            // Random rotation around each axis, small angles
            Vector3 randomEuler = new Vector3(
                UnityEngine.Random.Range(-10f, 10f),  // Roll (x)
                UnityEngine.Random.Range(-10f, 10f),  // Pitch (y)
                UnityEngine.Random.Range(-10f, 10f)   // Yaw (z)
            );
            // Convert to Quaternion
            Quaternion randomRotation = Quaternion.Euler(randomEuler);
            Quaternion<NED> rotationNed = randomRotation.To<NED>();
            Quaternion rotationNedUnity = rotationNed.ToUnityQuaternion();
            // Apply relative to current rotation
            Quaternion newRot = dynamics.GetQuaternionNED() * rotationNedUnity;

            return newRot;
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
            qInput = dynamics.GetQuaternionNED(); 
            velsInput = dynamics.GetVelsNED(); // 1x6
            sensor.AddObservation(dynamics.GetQuaternionNED()); // 1x4
            sensor.AddObservation(dynamics.GetVelsNED()); // 1x6
            
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
            
            /*
            var reward = ComputeReward();
            //print("REWARD:");
            //print(reward);
            if (float.IsNaN(reward))
            {
                Debug.Log("Warning! Reward NaN! Something went wrong!");
            }
            else
            {
                // Debug.Log(GetCumulativeReward()); //NB! NEVER LEAVE CONSTANT DEBUG MESSAGES ENABLED, It causes massive slowdowns
                AddReward(reward);
            }*/
            ContinousRewards();
            
            prevPos = currPos;
            prevActions = currActions;
            EpisodeInterrupted();
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
            // Inspo: https://github.com/martkartasev/SMARC-RL/blob/master/Assets/Scripts/SAMGeneralMovementLearningAgent.cs
            // We manually ensure the rewards never exceed the -1 : 1 range during an episode.
            // This is for stability in the neural networks.
            // We dont use normalization in the network inputs, and do it manually ourselves.
            // Doing it ourselves, we dont have to "learn" what the possible range of values is.
            // IF you do this manually, make sure to turn off normalization in the learning config file.

            // Distance to gate reward
            currPos = dynamics.GetPosNED();
            float currentDistToGate = Vector3.Distance(next2Gates[0], currPos);
            float clamped = Math.Clamp((maxDistance - currentDistToGate) / maxDistance, 0, 1);
            //print("claeemd: " + clamped);
            var reward = clamped;// / MaxStep * 0.8f; TODO: MaxStep is set in the scene for this script. Default 0 so will get error if used as default. ask mart what he has
            //print("max step *0.8: " + reward);
            //print(MaxStep);
            
            // TODO: should we implement target speed?
            
            // Facing gate / allign with target reward
            // Currently unused "align with target" reward. Currently insufficient observation for this, cant enable TODO: ask mart about this
            // reward += 0.xf * ((Vector3.Dot(targetObject.forward, body.transform.forward) + 1) * 0.5f); 
            
            // Time penalty
            reward += -0.5f; // / MaxStep;
            
            // Smooth actions reward
            //Vector<float> actionDiff = currActions - prevActions;
            //float actionDiffNorm = (float) actionDiff.L2Norm();
            
            return reward;
        }

        private void FixedUpdate()
        {
            if (dynamics.GetHeight() >= 0)
            {
                //Debug.LogError("AAAAJ TAK");
                AddReward(-1.0f);
                EndEpisode();
            }
            
           
        }

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

        private void ContinousRewards()
        {
            // r_progression
            float d_prev = Vector3.Distance(next2Gates[0], prevPos);
            currPos = dynamics.GetPosNED();
            float d_curr = Vector3.Distance(next2Gates[0], currPos);
            float r_prog = 6*lambda1 * (d_prev - d_curr);

            // r_perception TODO: implement
            Vector3 directionToGate = (NED.ConvertToRUF(next2Gates[0]) - NED.ConvertToRUF(currPos)).normalized;
            //Debug.DrawRay(brovPhysics.mainBody.transform.position, directionToGate * 2f, Color.red);
            //Debug.DrawRay(brovPhysics.mainBody.transform.position, brovPhysics.GetForwardUnitVec() * 2f, Color.blue);
            //directionToGate.y = 0;
            float angleToGate = Vector3.SignedAngle(dynamics.GetForwardUnitVec(), directionToGate, Vector3.up); // yaw angle to gate
            // TODO: this formula doesnt make sense??? from the report
		    float part = lambda3 * Mathf.Pow(angleToGate, 4); // NOTE: pow of 2 instead of 4 as in the report
            //float r_perc = lambda2 * Mathf.Exp(part);
            float r_perc = 0;
            
            // Facing gate / allign with target reward
            // Currently unused "align with target" reward. Currently insufficient observation for this, cant enable TODO: ask mart about this
            var r = checkpoints[iNextGate].transform.forward;
            var d = dynamics.GetForwardUnitVec();
            float r_allign = ((Vector3.Dot(r, d) + 1) * 0.5f);
            print(r_allign);
            
            // smoothness
            
            float diffSum = 0f;
            for (int i = 0; i < currActions.Count; i++) {
                diffSum += Mathf.Abs(currActions[i] - prevActions[i]);
            }
            float r_cmd2 = -1 * (diffSum / (2 * currActions.Count));
            print("SMOOOOOTH: " + r_cmd2);
            Vector<float> actionDiff = currActions - prevActions;

            float actionDiffNorm = (float) actionDiff.L2Norm();
            float magnitude = Mathf.Pow(actionDiffNorm, 2);
            float r_cmd = lambda5*magnitude;
            //print("CMD REWARD: " + magnitude);
            /*
            if (dynamics.GetHeight() >= 0)
            {
                //Debug.LogError("AAAAJ TAK");
                AddReward(-5.0f);
                EndEpisode();
                
            }*/
		
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
                AddReward(5f); 
                iNextGate = (iNextGate + 1) % gatePositions.Count;
                //print("next gate index: " + iNextGate);
                next2Gates[1] = gatePositions[iNextGate]; 
                next2Gates[0] = next2Gates[1];
                //Debug.Log("next gate: " + next2Gates[0]);
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