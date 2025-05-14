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
        
        void Start()
        {
            // ros stuff
            ros = ROSConnection.GetOrCreateInstance();
            //ros.Subscribe<Int32MultiArrayMsg>(topic, ReceiveControlOutput);
            ros.Subscribe<OverrideRCInMsg>(topic, ReceiveControlOutput);
        }

        public float[] GetRosControlOutput()
        {
            return dofControl;
        }

        void ReceiveControlOutput(OverrideRCInMsg msg)
        { // TODO: add mavros msg to unity or make so this actually recives Float64MultiArrayMsg
            // msg - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [1100, 1900])
            print("---RECIEVED CONTROL OUTPUT---");
            
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
            print("DOF CONTROLS RECIEVED");
            print(dofControl[0] + "     " + dofControl[1] + "     " + dofControl[2]);

        }
    }
}