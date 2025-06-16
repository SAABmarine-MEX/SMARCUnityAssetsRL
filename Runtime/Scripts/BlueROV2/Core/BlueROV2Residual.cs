using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Physics;
using DefaultNamespace.BlueROV2.SITL;
using System;
using DefaultNamespace.BlueROV2.ROS.Subscribers;
using BlueROV2.Physics;


namespace DefaultNamespace.BlueROV2.Core
{
    public class BlueROV2Residual : MonoBehaviour // TODO: combine this with BlueROV2.cs
    {
        [Header("Options")]
        
        [Tooltip("Toggles residual dynamic modelling inference")]
        public bool useResModel = true;
        private bool isQuerying = false;
        
        [Tooltip("Use ArduSub sitl or use scaled max tau control")]
        public bool useArdusub = true; // Bool to declare if to use ArduSub sitl or to use scaled max tau control
        
        [Tooltip("Used to simulate 'real'. Only used to validate residual training so keep false otherwise")]
        public bool isReal = false; //
        
        [Tooltip("Get actuation commands from ros")]
        public bool useRos = true;
        
        [Tooltip("Toggle tether dynamics")]
        public bool useTetherDynamics = true;
        [Tooltip("Toggle tether visuals")]
        public bool useTetherVisuals = true;
        
        
        [Header("BlueROV2 components")]
        // The three building blocks for the BlueROV2
        public BrovDynamics dynamics;
        public ArduSub sitl;
        public Tether tether;
        
        
        [Header("Residual modelling components")]
        public ResidualControl agent;
        // To give manual mode for executble with residual inference
        public ResidualPrepper resPrepper;
        public PythonModelHttpClient client;
        
        
        [Header("ROS components")]
        public Actuation6dof_Sub rosActuation;

        
        // map frame. To replicate standard ros map frame
        private GameObject map;
        
        // Input 
        private float[] dofControl = new float[] { 1500, 1500, 1500, 1500, 1500, 1500 };
        // Output
        private float[] bodyTau = new float[] { 0, 0, 0, 0, 0, 0 };
        
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
            
            // Ros actuation
            rosActuation = GetComponent<Actuation6dof_Sub>();
            if (rosActuation == null)
                Debug.LogError("Ros Actuation component not found on this GameObject.");
            
            // Tether
            tether = GetComponent<Tether>();
            if (tether == null)
                Debug.LogError("Tether component not found on this GameObject.");
            
            // Both dynamics and agent needs awareness of the map frame
            dynamics.Setup(map);
            agent.Setup(map);
            
            // Run update of dynamics in this script's FixedUpdate to give better understand of what is happening
            dynamics.allowFixedUpdate = false; 
            
            // TODO: maybe change so this could be changed when it is running
            tether.SetUseTetherDynamics(useTetherDynamics);
            tether.SetUseTetherVisuals(useTetherVisuals);
        }
        
        async void FixedUpdate()
        {
            if (isQuerying)
            {
                //print("Is querying");
                return;
            }
            
            // Get control output
            // 1. Get control output
            if (!useRos)
                dofControl = agent.GetScaledActions().ToArray();
            else
                dofControl = rosActuation.GetRosControlOutput(); // TODO: denna ordning blir fel från vad overriderc är i sättet den hanteras. roll och pitch är tvärt om

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
            
            // If isReal, meaning trying to simulate a "real" environment that differ from prior
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
                resPrepper.SetActions(dofControl);
                
                // Velocity 
                Vector<float> vels = dynamics.GetVelsNED();
                resPrepper.SetVelsNED(vels);
                
                // Gather prepped features
                float[] features = resPrepper.GatherFeatures();
                
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