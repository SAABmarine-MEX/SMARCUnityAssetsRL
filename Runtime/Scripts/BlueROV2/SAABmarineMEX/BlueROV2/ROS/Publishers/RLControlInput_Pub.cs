using UnityEngine;
using DefaultNamespace.BlueROV2.Control;
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
 * The purpose of this ros publisher is to publish rl control input.
 * It first needs to retrieve the rl control input from the RLControl component, and then publish it
 */

namespace DefaultNamespace.BlueROV2.ROS.Publishers
{
    public class RLControlInput_Pub : MonoBehaviour
    {
        //[RequireComponent(typeof(RLControl))]
        public RLController agent;
        
        // ros stuff
        ROSConnection ros;
        public string topic = "/RLControl/input";

        void Start()
        {
            agent = GetComponentInChildren<RLController>();
            
            // ros stuff
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<Float64MultiArrayMsg>(topic);
            InvokeRepeating("PublishControlInput", 0.02f, 0.02f); // TODO: make into freq variable
        }

        void PublishControlInput()
        {
            // Get control input
            List<float> controlInput = agent.GetControlInput();
            
            // Convert to double for stupid reason
            float[] inputArray = controlInput.ToArray();
            double[] doubleArray = Array.ConvertAll(inputArray, x => (double)x);
            
            // Create msg
            Float64MultiArrayMsg inputMsg = new Float64MultiArrayMsg
            {
                data = doubleArray, // TODO: why must this be double when it says float array
            };
            
            // Publish msg
            ros.Publish(topic, inputMsg);
        }
    }
}