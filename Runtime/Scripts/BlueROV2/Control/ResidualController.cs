using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Physics;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// This script is used from the residual modelling with mlagents llapi 

namespace DefaultNamespace.BlueROV2.Control
{
    public class ResidualController : Agent
    {
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
        }

        public override void OnEpisodeBegin() // TODO: could move this to Start() since this will not do any episodes since it is not for training, as of now
        {
            // Reset the Brov to its starting position
            Vector3 startPos = NED.ConvertToRUF(dynamics.GetPosNED()); // Start pos from the scene
            Quaternion startRot = Quaternion.Euler(0, 0, 0);
            dynamics.SetZeroVels();
            dynamics.SetPose(startPos, startRot);
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            /*
            * This method adds observations to the sensor of the agent. 
            * The observations are used as input to the neural network.
            */
            
            // State
            sensor.AddObservation(dynamics.GetPosNED());        // Position,    1x3
            sensor.AddObservation(dynamics.GetQuaternionNED()); // Orientation, 1x4
            sensor.AddObservation(dynamics.GetVelsNED());       // Velocities,  1x6
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // Actions then used in BlueROV2 core script
            ActionSegment<float> actionsSeg = actions.ContinuousActions;
            currActions = Vector<float>.Build.Dense(actionsSeg.Length, i => actionsSeg[i]);
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