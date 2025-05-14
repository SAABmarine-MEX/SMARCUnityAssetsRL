using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;

namespace DefaultNamespace.BlueROV2.Core
{
    public class BlueROV2Residual : MonoBehaviour
    {
        // The three building blocks for the BlueROV2
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public ResidualControl agent;
         
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        public bool isReal = false; //
        
        // map frame. To replicate standard ros map frame
        private GameObject map;
        
        float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
        
        void Awake()
        {
            map = GameObject.Find("map"); // map frame as in ros
            if (map != null){ Debug.Log("map found"); }
            
            // These are now set directly from the scene
            //dynamics = GetComponent<BrovDynamics>();
            //agent = GetComponent<RLController>();
            //sitl = GetComponent<ArduSub>();

            if (isReal)
            {
                // If real, then change the dynamics of this so that the prior scene then will do the residual modeling to match real
                // dynamics.Xuu = 142
            }
            
            // Both dynamics and agent needs awareness of the map frame
            dynamics.Setup(map);
            agent.Setup(map);
            
            // Run update of dynamics in this script's FixedUpdate to give better understand of what is happening
            dynamics.allowFixedUpdate = false; 
        }
        
        void FixedUpdate()
        {
            // Get control output
            float[] dofControl = agent.GetScaledActions().ToArray();
            print("CONTROLS");
            print(dofControl[0] + "     " + dofControl[1] + "     " + dofControl[2] + "     " + dofControl[3]);
            
            // Apply control output depending if want to use ardusub sitl or not
            if (useArdusub)
            {
                float[] thrustersPwm = sitl.SITL(dofControl);
                //print("THRUUUUST");
                //print(thrustersPwm[0]);
                bodyTau = dynamics.GetBodyTauFromThrusters(thrustersPwm);
            }
            else
            {
                float[] u = sitl.RCInput(dofControl);
                bodyTau = dynamics.GetBodyTauFromMaxTau(u);
            }
            
            // If isReal, meaning trying to simulate a "real" enviornment that differ from prior
            if (isReal)
            {
                for (int i = 0; i < bodyTau.Length; i++) { bodyTau[i] *= 1.5f; }
            }
            
            // Set generated tau to body
            dynamics.SetInputTauNED(bodyTau);
            
            // Update dynamics
            dynamics.UpdateDynamics();
        }
    }
}