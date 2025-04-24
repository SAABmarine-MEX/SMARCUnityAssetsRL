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

/*
 * The purpose of this ros subscriber is to get the control output
 * and then send that to the agent which will actuate it
 */

namespace DefaultNamespace.BlueROV2.ROS.Subscribers
{
    public class Actuation6dof_Sub : MonoBehaviour
    {
        //[RequireComponent(typeof(RLControl))]
        //public RLController agent;
        public Brov brov;
        
        // ros stuff
        ROSConnection ros;
        public string topic = "/brov/rl/output";
        
        void Start()
        {
            //agent = GetComponentInChildren<RLController>();
            brov = GetComponent<Brov>();
            
            // ros stuff
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Int32MultiArrayMsg>(topic, ReceiveControlOutput);
        }

        void ReceiveControlOutput(Int32MultiArrayMsg msg)
        { // TODO: add mavros msg to unity or make so this actually recives Float64MultiArrayMsg
            // msg - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [1100, 1900])
            print("---RECIEVED CONTROL OUTPUT---");
            
            float[] dofControl = new float[6];
            // With this way, it could practically take the whole overridercin msg eventhough everything above index 5 is useless
            for (int i = 0; i < dofControl.Length; i++) { dofControl[i] = (float) msg.data[i]; }
            
            brov.SetDofControl(dofControl);

        }
    }
}