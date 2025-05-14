using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Unity.Mathematics;
using UnityEngine.Android;

// TODO: implement delay

namespace DefaultNamespace.BlueROV2.SITL
{
    public class ArduSub : MonoBehaviour
    {
        private float _throttle_thrust_max = 1;
        
        int AP_MOTORS_MAX_NUM_MOTORS = 8;
        private float[] _thrust_rpyt_out;
        private float[] _thrust_rpyt_out_scaled;

        private Matrix<double> T_hat_transpose;
        private Matrix<double> T_hat_transpose2;
        private Matrix<double> T_hat_transpose3;

        void Start()
        {
            _thrust_rpyt_out = new float[AP_MOTORS_MAX_NUM_MOTORS];
            _thrust_rpyt_out_scaled = new float[AP_MOTORS_MAX_NUM_MOTORS];
            
            T_hat_transpose2 = DenseMatrix.OfArray(new double[,]
            { // from the rapport "modifications to ardusub"
                {-1,  1,  0,  0,  0,  1},
                {-1, -1,  0,  0,  0, -1},
                { 1,  1,  0,  0,  0, -1},
                { 1, -1,  0,  0,  0,  1},
                { 0,  0, 1,  1, -1,  0}, // NOTE: var fel här innan från i höstas!
                { 0,  0, 1, -1, -1,  0},
                { 0,  0, 1,  1,  1,  0},
                { 0,  0, 1, -1,  1,  0}, // NOTE: var fel här innan från i höstas!
            });
            
            T_hat_transpose3 = DenseMatrix.OfArray(new double[,]
            { // from unity last semester
                {-1,  1,  0,  0,  0,  1},
                {-1, -1,  0,  0,  0, -1},
                { 1,  1,  0,  0,  0, -1},
                { 1, -1,  0,  0,  0,  1},
                { 0,  0, -1,  1, -1,  0},
                { 0,  0, 1, -1, -1,  0},
                { 0,  0, 1,  1,  1,  0},
                { 0,  0, -1, -1,  1,  0},
            });
            
            T_hat_transpose = DenseMatrix.OfArray(new double[,] // TODO: is this T_trans för NED? för verkar funka med Z-down som positiv
            { // from mpc last semester. NOTE: this works best, gives right directions
                {1,  -1,  0,  0,  0,  -1},
                {1, 1,  0,  0,  0, 1},
                { -1,  -1,  0,  0,  0, 1},
                { -1, 1,  0,  0,  0,  -1},
                { 0,  0, -1,  1, 1,  0},
                { 0,  0, 1, 1, -1,  0},
                { 0,  0, 1,  -1,  1,  0},
                { 0,  0, -1, -1,  -1,  0},
            });
        }
        
        public float[] SITL(float[] dofInput)
        {
            /*
             * Input:
             * dofInput - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [1100, 1900]),
             * ordered as in mavros OverrideRCIn msg
             * 
             * Output:
             * mPwms - 8x1 motor pwms currently normilized [-1.0, 1.0]. See figure 3 for motor indexing in https://www.mdpi.com/2076-3417/14/17/7453#B16-applsci-14-07453
             */
            // Structure inspo from: https://www.mdpi.com/2076-3417/14/17/7453#B16-applsci-14-07453 
            float[] u = RCInput(dofInput);
            //print("SITL U:");
            //print(u[0] + " " + u[1] + " " + u[2] + " " + u[3] + " " + u[4] + " " + u[5]);
            
            float[] mHat = MotorCommand(u);
            //print("mHat: ");
            //print(mHat[0] + ", " + mHat[1] + ", " + mHat[2] + ", " + mHat[3] + ", " + mHat[4] + ", " + mHat[5] + ", " + mHat[6] + ", " + mHat[7]);
            
            float[] mPwms = RCOutput(mHat);

            return mPwms;

        }
        
        public float[] RCInput(float[] dofInput)
        {
            /*
             * Input:
             * dofInput - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [1100, 1900]),
             * ordered as in mavros OverrideRCIn msg
             *
             * Output:
             * u - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [-1.0, 1.0])
             */
            float[] u = new float[6];
            
            for (int i = 0; i < dofInput.Length; i++) // TODO: dofInput.Length == 6
            {
                u[i] = 2.0f * (dofInput[i] - 1100f) / 800f - 1.0f;
            }
            
            return u;
        }
        private float[] MotorCommand(float[] u)
        {
            /*
             * Input:
             * u - [roll, pitch, throttle (up/down), yaw, forward, lateral] (each between [-1.0, 1.0])
             *
             * Process:
             * Replicated ardusub's process to go from 6 dof control to 8 motor pwms.
             * Inspired by its code: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp
             * AP_Motors6DOF::output_armed_stabilizing_vectored_6dof
             * 
             * Output:
             * _thrust_rpyt_out - 8x1 motor pwms currently normilized [-1.0, 1.0]
             */
            int i;                                  // general purpose counter
            // TODO: also make sure it is integrated from the ros side aswell
            float   roll_thrust     = u[0];         // roll thrust input value, +/- 1.0
            float   pitch_thrust    = u[1];         // pitch thrust input value, +/- 1.0
            float   throttle_thrust = u[2];         // throttle thrust input value, +/- 1.0
            float   yaw_thrust      = u[3];         // yaw thrust input value, +/- 1.0
            float   forward_thrust  = u[4];         // forward thrust input value, +/- 1.0
            float   lateral_thrust  = u[5];         // lateral thrust input value, +/- 1.0
            
            float[] rpt_out = new float[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
            float[] yfl_out = new float[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
            float rpt_max;
            float yfl_max;
            
            // initialize limits flags
            bool limit_roll = false;
            bool limit_pitch = false;
            bool limit_yaw = false;
            bool limit_throttle_lower = false;
            bool limit_throttle_upper = false;
            
            // sanity check throttle is above zero and below current limited throttle
            // NOTE: not used currently as in ardusub, so could prob be removed so simplify
            if (throttle_thrust <= -_throttle_thrust_max) {
                throttle_thrust = -_throttle_thrust_max;
                limit_throttle_lower = true;
            }

            if (throttle_thrust >= _throttle_thrust_max) {
                throttle_thrust = _throttle_thrust_max;
                limit_throttle_upper = true;
            }
            
            // calculate roll, pitch and Throttle/heave for each motor (only used by vertical thrusters)
            rpt_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                //if (motor_enabled[i]) { // NOTE: this is for arm function in ardusub. not implemented in this sim
                    //rpt_out[i] = roll_thrust * _roll_factor[i] +
                    //             pitch_thrust * _pitch_factor[i] +
                    //             throttle_thrust * _throttle_factor[i];
                    //rpt_out[i] = roll_thrust * (float) T_hat_transpose[i, 2] +
                    //             pitch_thrust * (float) T_hat_transpose[i, 3] +
                    //             throttle_thrust * (float) T_hat_transpose[i, 4];
                    rpt_out[i] = throttle_thrust * (float) T_hat_transpose[i, 2] +
                                 roll_thrust * (float) T_hat_transpose[i, 3] +
                                 pitch_thrust * (float) T_hat_transpose[i, 4];
                    if (math.abs(rpt_out[i]) > rpt_max) {
                        rpt_max = math.abs(rpt_out[i]);
                    }
                //}
            }
            
            // calculate linear/yaw command for each motor (only used for translational thrusters)
            // linear factors should be 0.0 or 1.0 for now
            yfl_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                //if (motor_enabled[i]) { // NOTE: this is for arm function in ardusub. not implemented in this sim
                    //yfl_out[i] = yaw_thrust * _yaw_factor[i] +
                    //             forward_thrust * _forward_factor[i] +
                    //             lateral_thrust * _lateral_factor[i];
                    yfl_out[i] = forward_thrust * (float) T_hat_transpose[i, 0] +
                                 lateral_thrust * (float) T_hat_transpose[i, 1] +
                                 yaw_thrust * (float) T_hat_transpose[i, 5];
                    if (math.abs(yfl_out[i]) > yfl_max) {
                        yfl_max = math.abs(yfl_out[i]);
                    }
                //}
            }
            
            // Calculate final output for each motor and normalize if necessary
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                //if (motor_enabled[i]) { // NOTE: this is for arm function in ardusub. not implemented in this sim
                    //_thrust_rpyt_out[i] = Math.Clamp(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);

                    _thrust_rpyt_out[i] = Math.Clamp(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max,-1.0f,1.0f);
                    //_thrust_rpyt_out_scaled[i] = 1500 + 400*_thrust_rpyt_out[i];
                //}
            }
            return _thrust_rpyt_out;
        }

        private float[] RCOutput(float[] pwms)
        {
            /*
             * [-1.0, 1.0]
             */
            // Currently not scaled because pwm to force thruster function is normilazed and not scaled to [1100, 1900]
            // But in ardusub the pwms are scaled here
            // for-loop: u = 1500 + 400*_thrust_rpyt_out[i];
            float[] u = pwms;
            return u;
        }
    }
}