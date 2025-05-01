using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;
using MathNet.Numerics.LinearAlgebra;

/*
 * The purpose of this script is to give a dynamic way of sending control output and applying as body tau
 */


namespace DefaultNamespace.BlueROV2.Core
{
    public class Brov : MonoBehaviour
    {
        // Components
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public RLController agent; // Agent is only for training, not ros rl inference
        public Residual resModel;
        private GameObject map; // map frame. To replicate standard ros map frame
        
        // Flags
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        private bool useRLTraining = true; // TODO: make script override public varibles set in scene
        // TODO: problem with running inference from ros
        private bool useResModel = true;
        
        // Input 
        private float[] dofControl = new float[] { 1500, 1500, 1500, 1500, 1500, 1500 };
        // Output
        private float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
        void Awake()
        {
            // Get components
            // Dynamics
            dynamics = GetComponent<BrovDynamics>();
            if (dynamics == null)
                Debug.LogError("BrovDynamics component not found on this GameObject.");

            // SITL
            sitl = GetComponent<ArduSub>();
            if (sitl == null)
                Debug.LogError("ArduSub component not found on this GameObject.");

            // Agent
            Transform agentTransform = transform.Find("odom/base_link");
            if (agentTransform == null)
            {
                Debug.LogError("Could not find Transform 'odom/base_link'.");
            }
            else
            {
                agent = agentTransform.GetComponent<RLController>();
                if (agent == null)
                    Debug.LogError("RLController component not found on 'odom/base_link'.");
            }
            
            // Residual
            resModel = GetComponent<Residual>();
            if (resModel == null)
                Debug.LogError("Residual component not found on this GameObject.");
            
            // Map
            Transform current = transform.parent;
            while (current != null)
            {
                if (current.name == "map")
                {
                    Debug.Log("Found 'map' GameObject!");
                    map = current.gameObject;
                    break;
                }
                current = current.parent;
            }
            if (map == null) 
                Debug.LogError("'map' GameObject not found in parent hierarchy.");
            // Both dynamics and agent need awareness of the map frame
            dynamics.Setup(map);
            agent.Setup(map);
        }

        // Used by ros
        public void SetDofControl(float[] recievedDofControl) { dofControl = recievedDofControl; }
        
        void FixedUpdate()
        {
            // Get control output
            if (useRLTraining)
                dofControl = agent.GetScaledActions().ToArray();

            // Apply control output depending if want to use ardusub sitl or not
            if (useArdusub)
            {
                float[] thrustersPwm = sitl.SITL(dofControl);
                bodyTau = dynamics.SimulateFromThrusters(thrustersPwm);
            }
            else
            {
                float[] u = sitl.RCInput(dofControl);
                bodyTau = dynamics.SimulateFromMaxTau(u);
            }
            
            if (useResModel)
            {
                // Give action
                resModel.RecordActions(dofControl);
                
                // Give vels
                Vector<float> vels =  dynamics.GetVelsNED();
                resModel.GetVelsNED(vels);

                // Do res thing
                float[] aRes = resModel.GetResiduals();
                
                // Calculate tau given residual acc
                float[] tauRes = dynamics.CalculateResidualTau(aRes);
                dynamics.SetInputTauNED(tauRes);
            }
            
            // Apply generated tau to body
            dynamics.SetInputTauNED(bodyTau);
        }
    }
}