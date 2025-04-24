using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using Unity.Robotics.Core; //Clock
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using DefaultNamespace.BlueROV2.Physics;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DefaultNamespace.BlueROV2.ROS.Publishers.TF
{
    public class MapToBaselink_Pub : MonoBehaviour
    {
        ROSConnection ros;
        private string topicName = "/tf";
        
        public BrovDynamics dynamics;
        
        public GameObject mapFrame;      // Reference to the 'map' frame GameObject
        public GameObject baseLinkFrame; // Reference to the 'base_link' GameObject
        
        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<TFMessageMsg>(topicName);
            
        }

        void FixedUpdate()
        {
            // maybe not best way but i know these are map to baselink
            // TODO: use the gameboject frames for mor dynamic code
            Vector3 position = dynamics.GetPosNED();
            Quaternion rotation = dynamics.GetQuaternionNED();
            Quaternion<NED> rotationNed = rotation.To<NED>();
            Quaternion<ENU> rotationEnu = rotationNed.To<ENU>();
            
            // Convert to ENU
            Vector3 enu = new Vector3(position.y, position.x, -position.z);  // N→E, E→N, D→-U
            
            // Rotation to convert from NED to ENU (180° around X, then 90° around Z)
            Quaternion nedToEnu = Quaternion.Euler(180f, 0f, 90f);
            // Equivalent of [sqrt(2)/2, sqrt(2)/2, 0, 0]
            float s = Mathf.Sqrt(2f) / 2f;
            Quaternion qRot = new Quaternion(s, s, 0f, 0f);  // x, y, z, w in Unity

            //Quaternion<ENU> rotationNed = qRot * rotationNed;
            
            TransformStampedMsg tfStamped = new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeStamp(Clock.time),
                    frame_id = "map"
                },
                child_frame_id = "brov2heavy/base_link",
                transform = new TransformMsg
                {
                    translation = new Vector3Msg(enu.x, enu.y, enu.z),
                    rotation = new QuaternionMsg(rotationEnu.x, rotationEnu.y, rotationEnu.z, rotationEnu.w)
                },
            };
            

            TFMessageMsg msg = new TFMessageMsg
            {
                transforms = new TransformStampedMsg[] { tfStamped, },
            };
            ros.Publish(topicName, msg);
        }

    }
}