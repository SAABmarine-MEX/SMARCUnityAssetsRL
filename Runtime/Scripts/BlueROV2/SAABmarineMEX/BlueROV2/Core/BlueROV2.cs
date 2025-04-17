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
        
        void Awake()
        {
            dynamics = GetComponent<BrovDynamics>();
            
            agent = GetComponent<RLController>();
            agent.SetDynamics(dynamics);
            
            sitl = GetComponent<ArduSub>();
        }
        
        void FixedUpdate()
        {
            float[] dofControl = agent.GetScaledActions().ToArray();
            if (useArdusub)
            {
                float[] thrustersPwm = sitl.SITL(dofControl);
                dynamics.SimulateFromThrusters(thrustersPwm);
            }
            else
            {
                Debug.LogWarning("NOT YET IMPLEMENTED");
                dynamics.SimulateFromMaxTau(dofControl);
            }
        }
    }
}