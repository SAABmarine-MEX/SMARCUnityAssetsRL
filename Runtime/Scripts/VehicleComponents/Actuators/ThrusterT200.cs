using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils = DefaultNamespace.Utils;
using Force;  // MixedBody is in the Force namespace
using Unity.Mathematics;


using VehicleComponents.ROS.Core;

namespace VehicleComponents.Actuators
{
    public class ThrusterT200: LinkAttachment, IROSPublishable
    {
        [Header("Propeller")]
        public bool reverse = false;
        public double rpm;
        public double pwm;
        public float RPMMax = 100000;
        public float PWMMax = 1; // Normilized to [-1.0, 1.0] instead of actual [1100, 1900]
        public float RPMToForceMultiplier = 0.005f;

        [Header("Drone Propeller")]
        [Tooltip("Tick it for Drone and off for SAM/ROV")]
        public bool HoverDefault = false;
        public float NumPropellers = 4f;
        [Tooltip("should there be a torque")]
        public bool ApplyTorque = false;
        [Tooltip("direction of torque")]
        public bool TorqueUp = false;
        public double DefaultHoverRPM;

        public ArticulationBody baseLinkArticulationBody;
        public Rigidbody baseLinkRigidBody;
        private float c_tau_f = 8.004e-4f;
        private MixedBody baseLinkMixedBody; 
        
        public void SetRpm(double rpm)
        {
            this.rpm = Mathf.Clamp((float)rpm, -RPMMax, RPMMax);
            //if(hoverdefault) Debug.Log("setting rpm to: " + rpm);
        }

        public void SetPwm(double pwm)
        {
            this.pwm = Mathf.Clamp((float)pwm, -PWMMax, PWMMax);
        }

        public double PwmToForce2(double pwm)
        {
            // pwm: This one is nomrilizaed [-1, 1]
            double force = -140.3*math.pow(pwm,9)+389.9*math.pow(pwm,7)-404.1*math.pow(this.pwm,5)+176.0*math.pow(this.pwm,3)+8.9*this.pwm;
            return force;
        }
        public double PwmToForce(double pwm)
        {
            pwm = ((pwm + 1.0f) / 2.0f) * (1900 - 1100) + 1100;
            double kgf = 
                -3.04338931856672e-13f * Mathf.Pow((float) pwm, 5) +
                2.27813523978448e-9f  * Mathf.Pow((float) pwm, 4) +
                -6.73710647138884e-6f  * Mathf.Pow((float) pwm, 3) +
                0.00983670053385902f  * Mathf.Pow((float) pwm, 2) +
                -7.08023833982539f     * pwm +
                2003.55692021905f;

            return kgf * 9.80665f;
        }
        
        void Start()
        {
            baseLinkMixedBody = new MixedBody(baseLinkArticulationBody, baseLinkRigidBody);
            if(HoverDefault) InitializeRPMToStayAfloat();
        }

        void FixedUpdate()
        {
            var r = (float)rpm * RPMToForceMultiplier;
            // if(HoverDefault) Debug.Log("the value of 4xr is: " + r*4 );

            // Visualize the applied force
            
            //parentMixedBody.AddForceAtPosition((float) PwmToForce() * parentMixedBody.transform.forward,
              //                                     parentMixedBody.transform.position,
                //                                   ForceMode.Force);
            
            // Dont spin the props (which lets physics handle the torques and such) if we are applying manual
            // torque. This is useful for drones or vehicles where numerical things are known
            // and simulation is not wanted.
            if(ApplyTorque)   
            {
                int torque_sign = TorqueUp ? 1 : -1;
                float torque = torque_sign * c_tau_f * (float)r;
                Vector3 torqueVector = torque * transform.forward;
                parentMixedBody.AddTorque(torqueVector, ForceMode.Force);
            }
            else
            {
                int direction = reverse? -1 : 1;
                parentMixedBody.SetDriveTargetVelocity(ArticulationDriveAxis.X, direction*(float)rpm);
            }
        }

        private void InitializeRPMToStayAfloat()
        {
            // Calculate the required force to counteract gravity
            //float requiredForce = baseLinkArticulationBody.mass * Physics.gravity.magnitude;
            float requiredForce = baseLinkMixedBody.mass * Physics.gravity.magnitude;

            // Debug.Log("Required force to stay afloat: " + requiredForce);

            // Calculate the required RPM for each propeller
            float requiredForcePerProp = requiredForce/NumPropellers;
            float requiredRPM = requiredForcePerProp / RPMToForceMultiplier;
            DefaultHoverRPM = requiredRPM;

            // Set the initial RPM to each propeller
            SetRpm(requiredRPM);
        }

        public bool HasNewData()
        {
            return true;
        }
        
    }
}
