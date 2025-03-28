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
using DefaultNamespace;

public class BrovRLNode : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    ROSConnection ros;
    public string topicInput = "/controller_input"; // will publish state
    public string topicOutput = "/controller_output"; // will subscribe for actuation
    public string topicControlMode = "/controller_mode"; // will publish control mode
    private BrovPhysics brovPhysics;
    private BrovAgent brovAgent;
    public bool controlMode = true;
    
    public void OnTickChange(bool tick) // TODO: attribute no longer needed and change method name to more suitable for controlmodechange
    { // NOTE: not longer used since button was removed
        controlMode = !controlMode;
        BoolMsg controlModeChange = new BoolMsg
        {
            data = controlMode,
        };
        ros.Publish(topicControlMode, controlModeChange); // TODO: change so the python side understand its initial value. Either create seperate for first time or that it always sends. Reckon only first time would be smartest
    }
    void Start()
    {
        brovPhysics = GetComponent<BrovPhysics>(); // get brov physics component
        brovAgent = GetComponentInChildren<BrovAgent>();
        // ros stuff
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(topicInput); 
        ros.RegisterPublisher<BoolMsg>(topicControlMode);
        ros.Subscribe<Float64MultiArrayMsg>(topicOutput, RecieveOutput);
        InvokeRepeating("PublishState", 0.02f, 0.02f);
        InvokeRepeating("PublishMode", 0.02f, 0.02f);
    }

    void PublishMode()
    {
        //controlMode = !controlMode;
        // TODO: make it so it not constantly send. add a prev variable
        BoolMsg controlModeChange = new BoolMsg
        {
            data = controlMode,
        };
        ros.Publish(topicControlMode, controlModeChange);
    }
    void RecieveOutput(Float64MultiArrayMsg msg)
    {
        print("---RECIEVED CONTROL OUTPUT---");
        // This msg contains the forces outputed by the rl-model and will be applied to the brov
        Vector3 force = new Vector3((float) msg.data[0], (float) msg.data[1], (float) msg.data[2]);
        Vector3 torque = new Vector3((float) msg.data[3], (float) msg.data[4], (float) msg.data[5]);
        brovPhysics.SetInput(force, torque); // TODO: rename method to SetActuation
    }

    void PublishState()
    {
        // Get input for rl-model

        List<float> input = brovAgent.GetModelInput();
        print(input);
        float[] inputArray = input.ToArray();
        double[] doubleArray = Array.ConvertAll(inputArray, x => (double)x);
        Float64MultiArrayMsg stateArray = new Float64MultiArrayMsg
        {
            data = doubleArray, // TODO: why must this be double when it says float array
        };
        ros.Publish(topicInput, stateArray);
    }
}
