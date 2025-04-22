using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Physics;
using UnityEngine;


namespace DefaultNamespace.BlueROV2.Control
{
    public class ResidualControl : Agent
    {
        public bool isPrior = true; // Set in scene. if false - "real" dynamics
        public BrovDynamics dynamics;
        private GameObject map;
        public TeleopController teleopController;
        
        Vector<float> currActions = Vector<float>.Build.Dense(6, 0f);

        public void Setup(GameObject mapframe)
        {
            map = mapframe;
        }
        void Start()
        {
            if (dynamics == null)
            {
                Debug.LogError("dynamics not set");
            }
            map = GameObject.Find("map"); // Map frame as in ros
            
        }

        public override void OnEpisodeBegin()
        {
            // Reset the Brov to its starting position TODO: later, make the starting position more random
            Vector3 localStartPos = new Vector3(0.0f, 0.0f, 0.0f); // TODO: make it relative to the same origin as it will be irl
            Quaternion localStartRot = Quaternion.Euler(0, 0, 0);
            dynamics.SetZeroVels();
            dynamics.SetPose(localStartPos, localStartRot);
            
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
            
            // State
            sensor.AddObservation(dynamics.GetPosNED());        // Position,    1x3
            //print("POS:");
            //var pos = dynamics.GetPosNED();
            //print(pos.x + "," + pos.y + "," + pos.z);
            sensor.AddObservation(dynamics.GetQuaternionNED()); // Orientation, 1x4
            //var q = dynamics.GetQuaternionNED();
            //print("QUATERNIONS:");
            //print(q.x + "," + q.y + "," + q.z + "," + q.w);
            sensor.AddObservation(dynamics.GetVelsNED());       // Velocities,  1x6
            var vels = dynamics.GetVelsNED();
            //print("VELS:");
            //print(vels[3] + "       " + "       "+ vels[4] + "      "  + vels[5]);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            //print("GET ACTIOOOOOONS");
            ActionSegment<float> actionsSeg = actions.ContinuousActions;
            currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]);
            //print(currActions);
        }
        
        public override void Heuristic(in ActionBuffers actionsOut)
        {
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
                scaledActions[i] = ((actions[i] + 1f) * 0.5f) * (1900f - 1100f) + 1100f;
            }
            return scaledActions;
        }

        public Vector<float> GetScaledActions()
        {
            return ScaleActions(currActions);
        }
    }
}