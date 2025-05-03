using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.ROS.Subscribers;
using DefaultNamespace.BlueROV2.SITL;
using MathNet.Numerics.LinearAlgebra;

/*
 * The purpose of this script is to give a dynamic way of sending control output and applying as body tau to the BlueROV2
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
        public Actuation6dof_Sub rosActuation;
        
        // Flags
        // If to use ArduSub sitl or to use scaled max tau control
        public bool useArdusub = true;
        // If to use training pipeline or ros inference
        private bool useRLTraining = true;
        // If to use residual inference or not
        private bool useResModel = false;
        
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
            
            // Residual
            resModel = GetComponent<Residual>();
            if (resModel == null)
                Debug.LogError("Residual component not found on this GameObject.");
            
            // Ros actuation
            rosActuation = GetComponent<Actuation6dof_Sub>();
            if (rosActuation == null)
                Debug.LogError("Ros Actuation component not found on this GameObject.");
            
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

            // Run update of dynamics in this script's FixedUpdate to give better understand of what is happening
            dynamics.allowFixedUpdate = false; 
        }
        
        void FixedUpdate()
        {
            // 1. Get control output
            if (useRLTraining)
                dofControl = agent.GetScaledActions().ToArray();
            else
                dofControl = rosActuation.GetRosControlOutput();

            
            // 2. Apply control output depending if want to use ardusub sitl or not
            if (useArdusub)
            {
                float[] thrustersPwm = sitl.SITL(dofControl);
                bodyTau = dynamics.GetBodyTauFromThrusters(thrustersPwm);
            }
            else
            {
                float[] u = sitl.RCInput(dofControl);
                bodyTau = dynamics.GetBodyTauFromMaxTau(u);
            }
            
            
            // 3. Add generated tau to body
            dynamics.AddInputTauNED(bodyTau);
            
            
            // 4. If residual inference, add residual tau as well
            if (useResModel)
            {
                // Set actions
                resModel.SetActions(dofControl);
                
                // Set vels
                Vector<float> vels = dynamics.GetVelsNED();
                resModel.SetVelsNED(vels);

                // Do res thing
                float[] aRes = resModel.GetResiduals();
                
                // Calculate tau given residual acc
                float[] tauRes = dynamics.CalculateResidualTau(aRes);
                
                // Add the residual tau
                dynamics.AddInputTauNED(tauRes);
            }
            
            
            // 5. Update dynamics
            dynamics.UpdateDynamics();
            
        }
    }
}