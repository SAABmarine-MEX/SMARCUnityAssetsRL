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
            
            // Get dynamics
            dynamics = GetComponent<BrovDynamics>();
            if (dynamics == null)
                Debug.LogError("BrovDynamics component not found on this GameObject.");
            
            // Get map
            Transform current = transform.parent;
            while (current != null)
            {
                Debug.Log("Checking: " + current.name);
    
                if (current.name == "map")
                {
                    Debug.Log("Found 'map' GameObject!");
                    mapFrame = current.gameObject;
                    Debug.Log("Parent of " + current.name + " is: " + (current.parent != null ? current.parent.name : "null"));
                    break;
                }

                current = current.parent;
            }
            if (mapFrame == null)
            {
                Debug.LogWarning("'map' GameObject not found in parent hierarchy.");
            }
            if (mapFrame != null){ Debug.Log("map found"); }
            
            // Get baselink frame
            baseLinkFrame = transform.Find("odom/base_link").gameObject;
        }

        void FixedUpdate()
        {
            // maybe not best way but i know these are map to baselink
            // TODO: use the gameboject frames for mor dynamic code
            
            // Convert NED pos to ENU pos
            Vector3 positionNED = dynamics.GetPosNED();
            Vector3 posRUF = NED.ConvertToRUF(positionNED);
            Vector3<FLU> positionENU = posRUF.To<FLU>();
            
            // Convert NED orientation to ENU orientation
            Quaternion qNED = dynamics.GetQuaternionNED();
            Quaternion qRUF = NED.ConvertToRUF(qNED);
            Quaternion<FLU> qENU = qRUF.To<FLU>();
            
            TransformStampedMsg tfStamped = new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeStamp(Clock.time),
                    frame_id = "mocap" // before "map"
                },
                child_frame_id = "saab_rov", // before "brov2heavy/base_link" TODO: call ..._sim instead
                transform = new TransformMsg
                {
                    translation = new Vector3Msg(positionENU.x, positionENU.y, positionENU.z),
                    rotation = new QuaternionMsg(qENU.x, qENU.y, qENU.z, qENU.w)
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