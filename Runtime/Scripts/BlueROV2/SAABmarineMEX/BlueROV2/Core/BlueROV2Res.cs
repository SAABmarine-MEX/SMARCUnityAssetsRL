using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;
using MathNet.Numerics.LinearAlgebra;


namespace DefaultNamespace.BlueROV2.Core
{
    public class BrovRes : MonoBehaviour
    {
        // The three building blocks for the BlueROV2
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public RLController agent; // Agent is only for training, not ros rl inference
        public Residual resModel;
        
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        private bool useRLTraining = true; // TODO: make script override public varibles set in scene
        // TODO: problem with running inference from ros
        private bool useResModel = false;
        
        // map frame. To replicate standard ros map frame
        private GameObject map;
        
        float[] dofControl = new float[] { 1500, 1500, 1500, 1500, 1500, 1500 };
        float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
        
        void Awake()
        {
            map = GameObject.Find("map");
            if (map != null){ Debug.Log("map found"); }
            
            // Both dynamics and agent need awareness of the map frame
            dynamics.Setup(map);
            agent.Setup(map);
        }

        public void SetDofControl(float[] recievedDofControl) // Used like a interrupted by ros
        {
            //print("brov side received dof control-------------");
            //print(recievedDofControl[0] + "     " + recievedDofControl[1] + "     " + recievedDofControl[2]);
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
                print("UNIMPLEMENTED"); // TODO
                //float[] u = sitl.RCInput(dofControl);
                //bodyTau = dynamics.SimulateFromMaxTau(u);
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