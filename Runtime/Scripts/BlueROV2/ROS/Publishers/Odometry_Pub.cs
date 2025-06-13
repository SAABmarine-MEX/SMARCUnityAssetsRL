using UnityEngine;
using DefaultNamespace.BlueROV2.Physics;
using UnityEngine;
using UnityEditor;
using Unity.Mathematics;
using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using DefaultNamespace.LookUpTable;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.Core; //Clock
using RosMessageTypes.Std;
using UnityEngine.UIElements;
using VehicleComponents.Actuators;

// TODO: use smarc's script instead!
namespace DefaultNamespace.BlueROV2.ROS.Publishers
{
    public class Odometry_Pub : MonoBehaviour
    {
        public BrovDynamics dynamics;
        
        // ros stuff
        ROSConnection ros;
        public string topic = "/brov/sim/odometry";
        public string parent_frame_id = "map";
        public string the_child_frame_id = "base_link_gt";
        
        void Start()
        {
            dynamics = GetComponent<BrovDynamics>();
            
            // ros stuff
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<OdometryMsg>(topic);
            InvokeRepeating("PublishOdometry", 0.02f, 0.02f); // TODO: make into freq variable
        }

        void PublishOdometry()
        {
            //print("PUUUUUUUUUUUUUUUB");
            // Can't do built in attribute like header.stamp so need to build up individually... TODO: you can by nesting, fix
            // Header 
            HeaderMsg headerMsg = new HeaderMsg
            {
                stamp = new TimeStamp(Clock.time),
                frame_id = parent_frame_id,
            };
            
            
            // Building pose with cov msg
            // Position
            var position = dynamics.GetPosNED();
            PointMsg positionMsg = new PointMsg
            {
                x = position.x, y = position.y, z = position.z,
            };
            
            // Quaternion
            var q = dynamics.GetQuaternionNED();
            QuaternionMsg quaternionMsg = new QuaternionMsg
            {
                w = q.w, x = q.x, y = q.y, z = q.z,
            };
            
            // Pose 
            PoseMsg poseMsg = new PoseMsg
            {
                position = positionMsg, orientation = quaternionMsg,
            };
            
            PoseWithCovarianceMsg poseWithCovMsg = new PoseWithCovarianceMsg
            {
                pose = poseMsg,
            };
            
            
            // Building twist with cov msg
            // Linear velocity
            var linVel = dynamics.GetLinVelsNED();
            Vector3Msg linVelMsg = new Vector3Msg
            {
                x = linVel.x, y = linVel.y, z = linVel.z,
            };
            
            // Angular velocity
            var angVel = dynamics.GetAngVelsNED();
            Vector3Msg angVelMsg = new Vector3Msg
            {
                x = angVel.x, y = angVel.y, z = angVel.z,
            };

            
            // Twist 
            TwistMsg twistMsg = new TwistMsg
            {
                linear = linVelMsg, angular = angVelMsg,
            };

            TwistWithCovarianceMsg twistWithCovMsg = new TwistWithCovarianceMsg
            {
                twist = twistMsg,
            };
            
            
            // Odometry
            OdometryMsg odometryMsg = new OdometryMsg
            {
                header = headerMsg,
                pose = poseWithCovMsg,
                twist = twistWithCovMsg,
                child_frame_id = the_child_frame_id,
            };
            
            ros.Publish(topic, odometryMsg);
        }
    }
}