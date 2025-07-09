using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using DefaultNamespace.BlueROV2.Physics;


namespace DefaultNamespace.BlueROV2.ROS.Subscribers
{
    public class InitPoseSubscriber : MonoBehaviour
    {
        private string topic = "/initial_pose";
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
            Debug.Log("HÄÄÄR");
            Debug.Log(position);
            
            // TODO: maybe also get velocities, but only pos for now since it also most often start still "ish" from the bags
            
            //dynamics.SetInputTauNED(new float[] {0f, 0f, 0f, 0f, 0f, 0f});
            //dynamics.SetZeroVels();
            dynamics.SetPoseNed(position, rotation);
            //hasInitialized = true;
        }
    }
}