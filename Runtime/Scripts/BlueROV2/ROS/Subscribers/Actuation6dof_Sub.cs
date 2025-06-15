using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
using DefaultNamespace.BlueROV2.Core;
using UnityEngine;
using UnityEditor;
using Unity.Mathematics;
using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using DefaultNamespace.LookUpTable;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using UnityEngine.UIElements;
using VehicleComponents.Actuators;
using RosMessageTypes.Mavros;


/*
 * The purpose of this ros subscriber is to get the control output
 * and then send that to the agent which will actuate it
 */

namespace DefaultNamespace.BlueROV2.ROS.Subscribers
{
    public class Actuation6dof_Sub : MonoBehaviour
    {
        //[RequireComponent(typeof(RLControl))]
        
        private float[] dofControl = new float[] { 1500, 1500, 1500, 1500, 1500, 1500 };
        
        // ros stuff
        ROSConnection ros;
        //public string topic = "/brov/rl/output";
        private string topic = "/mavros/rc/override";
        
        private float timeoutDuration = 0.5f; // in seconds
        private float lastMessageTime;
        private bool isReset = false;
        
        void Start()
        {
            // ros stuff
            ros = ROSConnection.GetOrCreateInstance();
            //ros.Subscribe<Int32MultiArrayMsg>(topic, ReceiveControlOutput);
            ros.Subscribe<OverrideRCInMsg>(topic, ReceiveControlOutput);
            
            lastMessageTime = Time.time;
        }

        public float[] GetRosControlOutput()
        {
            return dofControl;
        }

        void FixedUpdate()
        {
            // TODO: kanske implementera i sitl istället och se hur ardusub gör det
            // If no new message for a while, reset control
            if (Time.time - lastMessageTime > timeoutDuration && !isReset)
            {
                ResetControlToNeutral();
                isReset = true;
            }
        }

        void ReceiveControlOutput(OverrideRCInMsg msg)
        {
            lastMessageTime = Time.time;
            isReset = false;

            // msg - [pitch, roll, throttle (up/down), yaw, forward, lateral] (each between [1100, 1900])
            //print("---RECIEVED CONTROL OUTPUT---");
            
            // With this way, it could practically take the whole overridercin msg eventhough everything above index 5 is useless
            for (int i = 0; i < dofControl.Length; i++)
            {
                // xnew​=1900−(x−1100)
                if (i==2) { dofControl[i] =  1900f - ( (float) msg.channels[i] - 1100f); } // invert z to be NED. ugly but becase T_transpose in ArduSub.cs is NEDed. just quick fix
                else
                {
                    dofControl[i] = (float) msg.channels[i];
                }
            }
            (dofControl[0], dofControl[1]) = (dofControl[1], dofControl[0]); // Swap roll and pitch to work with the rest of the code structure, especially important in sitl code
            //print("DOF CONTROLS RECIEVED");
            //print(dofControl[0] + "     " + dofControl[1] + "     " + dofControl[2]);

        }
        
        void ResetControlToNeutral()
        {
            for (int i = 0; i < dofControl.Length; i++)
            {
                dofControl[i] = 1500f;
            }
            print("DOF CONTROLS RESET TO NEUTRAL");
        }

    }
}