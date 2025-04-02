/*
 * The TeleopController class is responsible for controlling the robot during teleop mode. It is responsible for reading the joystick input and sending the appropriate commands to the robot.
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
    
    public Vector<float> GetTeleopInput()
    {
        Vector<float> input = Vector<float>.Build.Dense(6, 0f); 
        // NOTE: This is in NED frame
        // x
        if (Input.GetKey(KeyCode.W))
        {
            input[0] += 0.5f; //floorAndRoofCheck(input[0]+teleopSense);
        }
        if (Input.GetKey(KeyCode.S))
        {
            input[0] -= 0.5f; // floorAndRoofCheck(input[0]-teleopSense);
        }
        // y
        if (Input.GetKey(KeyCode.D))
        {
            input[1] += 0.5f; // floorAndRoofCheck(input[1]+teleopSense);
        }
        if (Input.GetKey(KeyCode.A)) 
        {
            input[1] -= 0.5f; //floorAndRoofCheck(input[1]-teleopSense);
        }
        // z. NOTE: In NED, positive z is down
        if (Input.GetKey(KeyCode.Space))
        {
            input[2] -= 0.5f; //floorAndRoofCheck(input[2]-teleopSense);
        }
        if (Input.GetKey(KeyCode.LeftShift))
        {
            input[2] += 0.5f; //floorAndRoofCheck(input[2]+teleopSense);
        }
        // yaw
        if (Input.GetKey(KeyCode.Q))
        {
            input[5] -= 0.5f;//floorAndRoofCheck(input[5]-teleopSense);
        }
        if (Input.GetKey(KeyCode.E))
        {
            input[5] += 0.5f;//floorAndRoofCheck(input[5]+teleopSense);
        }
        // pitch
        if (Input.GetKey(KeyCode.X))
        {
            input[4] += 0.5f; // floorAndRoofCheck(input[4]+teleopSense);
        }
        // roll
        if (Input.GetKey(KeyCode.C))
        {
            input[3] += 0.5f;//floorAndRoofCheck(input[3]+teleopSense);
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
