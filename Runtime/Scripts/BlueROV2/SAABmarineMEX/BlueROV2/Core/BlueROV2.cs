using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.ML;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;

namespace DefaultNamespace.BlueROV2.Core
{
    public class BlueROV2 : MonoBehaviour
    {
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public RLController agent;
        
        public bool useArdusub = true;
        
        private GameObject map;
        
        void Awake()
        {
            map = GameObject.Find("map"); // map frame as in ros
            if (map != null){ Debug.Log("map found"); }
            
            //dynamics = GetComponent<BrovDynamics>();
            dynamics.Setup(map);
            agent.Setup(map);
            
            //agent = GetComponent<RLController>();
            //agent.SetDynamics(dynamics);

            //sitl = GetComponent<ArduSub>();
        }
        
        void FixedUpdate()
        {
            float[] dofControl = agent.GetScaledActions().ToArray();
            print("DOF CONTROL");
            print(dofControl[0] + " " + dofControl[1] + " " + dofControl[2] + " " + dofControl[3] + " " + dofControl[4] + " " + dofControl[5]);
            
            if (useArdusub)
            {
                float[] thrustersPwm = sitl.SITL(dofControl);
                dynamics.SimulateFromThrusters(thrustersPwm);
            }
            else
            {
                float[] u = sitl.RCInput(dofControl);
                dynamics.SimulateFromMaxTau(u);
            }
        }
    }
}