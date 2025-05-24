using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;
using System;

namespace DefaultNamespace.BlueROV2.Core
{
    public class BlueROV2Residual : MonoBehaviour
    {
        // The three building blocks for the BlueROV2
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public ResidualControl agent;
        
        // To give manual mode for executble with residual inference
        public ResidualPrepper resModel;
        public PythonModelHttpClient client;
        // If to use residual inference or not
        public bool useResModel = true;
        private bool isQuerying = false;
         
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        public bool isReal = false; //
        
        // map frame. To replicate standard ros map frame
        private GameObject map;
        
        float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
        
        void Awake()
        {
            map = GameObject.Find("map"); // map frame as in ros
            if (map != null){ Debug.Log("map found"); }
            
            /*
            // Residual prepper
            resModel = GetComponent<ResidualPrepper>();
            if (resModel == null)
                Debug.LogError("Residual component not found on this GameObject.");

            // Residual client
            client = GetComponent<PythonModelHttpClient>();
            if (client == null)
                Debug.LogError("Client not found on this GameObject.");
            */
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
        
        async void FixedUpdate()
        {
            if (isQuerying)
            {
                //print("Is querying");
                return;
            }
            
            // Get control output
            float[] dofControl = agent.GetScaledActions().ToArray();
            //print("CONTROLS");
            //print(dofControl[0] + "     " + dofControl[1] + "     " + dofControl[2] + "     " + dofControl[3]);
            
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
            
            // If residual inference, add residual tau as well
            if (useResModel)
            {
                // Residual prepping
                // Acceleration
                resModel.SetActions(dofControl);
                
                // Velocity 
                Vector<float> vels = dynamics.GetVelsNED();
                resModel.SetVelsNED(vels);
                
                // Gather prepped features
                float[] features = resModel.GatherFeatures();
                
                // Query residual inference
                isQuerying = true;
                float[] residuals = new float[] {};
                try
                {
                    //print("START QUERYING");
                    residuals = await client.QueryResidualsAsync(features);
                    //print("Got residuals: " + string.Join(",", residuals));
                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("Error querying model: " + ex);
                }
                finally
                {
                    //print("DONE QUERYING");
                    
                    // Calculate tau given residual acc
                    float[] tauRes = dynamics.CalculateResidualTau(residuals);
                    
                    // Add the residual tau
                    dynamics.AddInputTauNED(tauRes);
                    isQuerying = false;
                }
            }
            
            // Update dynamics
            dynamics.UpdateDynamics();
        }
    }
}