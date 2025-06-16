/*
 * The TeleopController class is responsible for controlling the robot during teleop mode. It is responsible for reading the keyboard input and sending the appropriate commands to the robot.
 * It shall control the robot by giving forces to a rigid body in all 6 dof, 3 forces and 3 torques.
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using MathNet.Numerics.LinearAlgebra;

public class TeleopController : MonoBehaviour
{
    //private Vector<float> input = Vector<float>.Build.Dense(6, 0f); // 6 dof, 3 forces and 3 torques. Each will be in the range [-1, 1]. [x,y,z,roll,pitch,yaw]
    private float teleopSense = 0.1f; // This must be within [-1, 1] since that is the min and max for the controller
    private float scaleFactor = 1f;

    private void UpdateScaling()
    {
        if (Input.GetKey(KeyCode.N))
        {
            scaleFactor -= teleopSense;
        }

        if (Input.GetKey(KeyCode.P))
        {
            scaleFactor += teleopSense;
        }
        if (scaleFactor > 1){ scaleFactor = 1; }
        if (scaleFactor < -1) { scaleFactor = -1; }
    }
    public Vector<float> GetTeleopInput()
    {
        UpdateScaling();
        Vector<float> input = Vector<float>.Build.Dense(6, 0f); 
        // NOTE: This is in NED frame
        // overridercin structure
        // x
        if (Input.GetKey(KeyCode.W))
        {
            input[4] += 1f * scaleFactor; //floorAndRoofCheck(input[0]+teleopSense);
        }
        if (Input.GetKey(KeyCode.S))
        {
            input[4] -= 1f * scaleFactor; // floorAndRoofCheck(input[0]-teleopSense);
        }
        // y
        if (Input.GetKey(KeyCode.D))
        {
            input[5] += 1f * scaleFactor; // floorAndRoofCheck(input[1]+teleopSense);
        }
        if (Input.GetKey(KeyCode.A)) 
        {
            input[5] -= 1f * scaleFactor; //floorAndRoofCheck(input[1]-teleopSense);
        }
        // z. NOTE: In NED, positive z is down // TODO: to match overridercin, this must be up. or flip z else where
        if (Input.GetKey(KeyCode.Space))
        {
            input[2] -= 1f * scaleFactor; //floorAndRoofCheck(input[2]-teleopSense);
        }
        if (Input.GetKey(KeyCode.LeftShift))
        {
            input[2] += 1f * scaleFactor; //floorAndRoofCheck(input[2]+teleopSense);
        }
        // yaw
        if (Input.GetKey(KeyCode.Q))
        {
            input[3] -= 1f * scaleFactor;//floorAndRoofCheck(input[5]-teleopSense);
        }
        if (Input.GetKey(KeyCode.E))
        {
            input[3] += 1f * scaleFactor;//floorAndRoofCheck(input[5]+teleopSense);
        }
        // pitch
        if (Input.GetKey(KeyCode.X))
        {
            input[0] += 1f * scaleFactor; // floorAndRoofCheck(input[4]+teleopSense);
        }
        // roll
        if (Input.GetKey(KeyCode.C))
        {
            input[1] += 1f * scaleFactor;//floorAndRoofCheck(input[3]+teleopSense);
        }
        //else{ input = Vector<float>.Build.Dense(6, 0f); }
      return input;
    }
    private float floorAndRoofCheck(float input)
    {
        if(input > 1f) { input = 1f; }
        else if(input < -1f) { input = -1f; }
        return input;
    }
}
