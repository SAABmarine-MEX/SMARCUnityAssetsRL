using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using DefaultNamespace.BlueROV2.Physics;


// HÄMTAR POSE OCH TWIST FRÅN ETT RESPEKTIVE TOPIC
/*
namespace DefaultNamespace.BlueROV2.ROS.Subscribers
{
    public class YawOverrideSubscriber : MonoBehaviour
    {
        public string poseTopic = "/pose_real";
        public string twistTopic = "/pose_real";

        private ROSConnection ros;
        
        public BrovDynamics dynamics;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();

            ros.Subscribe<PoseStampedMsg>(poseTopic, ApplyPose);
            ros.Subscribe<TwistMsg>(twistTopic, ApplyTwist);
        }

        void ApplyPose(PoseStampedMsg msg)
        {
            Vector3 position = new Vector3(
                (float)msg.pose.position.x,
                (float)msg.pose.position.y,
                (float)msg.pose.position.z
            );

            Quaternion rotation = new Quaternion(
                (float)msg.pose.orientation.x,
                (float)msg.pose.orientation.y,
                (float)msg.pose.orientation.z,
                (float)msg.pose.orientation.w
            );
            
            dynamics.SetPoseNed(position, rotation);
        }

        void ApplyTwist(TwistMsg msg)
        {
            Vector3 linVel = new Vector3(
                (float)msg.linear.x,
                (float)msg.linear.y,
                (float)msg.linear.z
            );
            
            Vector3 angVel = new Vector3(
                (float)msg.angular.x,
                (float)msg.angular.y,
                (float)msg.angular.z
            );
            
            dynamics.SetVelsNed(linVel, angVel);
        }
    }
}
*/

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using DefaultNamespace.BlueROV2.Physics;
using MathNet.Numerics.LinearAlgebra;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;




namespace DefaultNamespace.BlueROV2.ROS.Subscribers
{
   public class YawOverride_Sub : MonoBehaviour
   {
       private string topic = "/pose_real";
       //private string topic = "/initial_pose";
       public BrovDynamics dynamics;


       public bool useInitPose = true;
       private bool hasInitialized = false;
       
       ROSConnection ros;
       
       void Start()
       {
           // ros stuff
           ros = ROSConnection.GetOrCreateInstance();
           //ros.Subscribe<Int32MultiArrayMsg>(topic, ReceiveControlOutput);
           ros.Subscribe<PoseStampedMsg>(topic, ApplyInitialPose);
       }

       void ApplyInitialPose(PoseStampedMsg msg)
       {
           if (hasInitialized || dynamics == null) return;
           
           Vector3 realPosition = new Vector3(
               (float)msg.pose.position.x,
               (float)msg.pose.position.y,
               (float)msg.pose.position.z
           );

           Quaternion realRotation = new Quaternion(
               (float)msg.pose.orientation.x,
               (float)msg.pose.orientation.y,
               (float)msg.pose.orientation.z,
               (float)msg.pose.orientation.w
           );
           
           // TODO: maybe also get velocities, but only pos for now since it also most often start still "ish" from the bags
           
           //dynamics.SetInputTauNED(new float[] {0f, 0f, 0f, 0f, 0f, 0f});
           //dynamics.SetZeroVels();
           
           
           Vector<float> currPoseNED = dynamics.GetPoseNED();
           Vector3 currPosNED = new Vector3(currPoseNED[0], currPoseNED[1], currPoseNED[2]);
           Quaternion currRotationNED = new Quaternion(currPoseNED[3], currPoseNED[4], currPoseNED[5], currPoseNED[6]);
           
           Vector3 eulerNewYawNED = new Vector3(  
               currRotationNED.eulerAngles.x,
               currRotationNED.eulerAngles.y,
               realRotation.eulerAngles.z
               );
           Vector3 eulerNewYaw = FRD.ConvertAngularVelocityToRUF(eulerNewYawNED);
           Quaternion rotNewYaw = Quaternion.Euler(eulerNewYaw);
           Quaternion rotNewYawNED = NED.ConvertFromRUF(rotNewYaw);
           dynamics.SetPoseNed(currPosNED, rotNewYawNED);
       }
   }
}
