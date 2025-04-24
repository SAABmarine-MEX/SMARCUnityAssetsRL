using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;


namespace DefaultNamespace.BlueROV2.Core
{
    public class Brov : MonoBehaviour
    {
        // The three building blocks for the BlueROV2
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public RLController agent; // Agent is only for training, not ros rl inference
        
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        public bool useRLTraining = true;
        
        // map frame. To replicate standard ros map frame
        private GameObject map;
        
        float[] dofControl = new float[] { 0, 0, 0, 0, 0, 0 };
        
        float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
        
        void Awake()
        {
            map = GameObject.Find("map");
            if (map != null){ Debug.Log("map found"); }
            
            // These are now set directly from the scene
            //dynamics = GetComponent<BrovDynamics>();
            //agent = GetComponent<RLController>();
            //sitl = GetComponent<ArduSub>();
            
            // Both dynamics and agent needs awareness of the map frame
            dynamics.Setup(map);
            agent.Setup(map);
        }

        public void SetDofControl(float[] recievedDofControl) // Used like a interrupted by ros
        {
            dofControl = recievedDofControl;
        }
        
        void FixedUpdate()
        {
            // Get control output
            
            if (useRLTraining)
            {
                dofControl = agent.GetScaledActions().ToArray();
                
            }
            
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
            
            // Apply generated tau to body
            dynamics.SetInputTauNED(bodyTau);
        }
    }
}