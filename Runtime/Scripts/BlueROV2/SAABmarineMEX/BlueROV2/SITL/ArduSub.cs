using System;
using Codice.Client.BaseCommands;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Unity.Mathematics;

namespace DefaultNamespace.BlueROV2.SITL
{
    public class ArduSub
    {
        private float _throttle_thrust_max = 1;
        
        int AP_MOTORS_MAX_NUM_MOTORS = 8;
        private float[] _thrust_rpyt_out;
        private float[] _thrust_rpyt_out_scaled;

        private Matrix<double> T_hat_transpose;

        void Start()
        {
            T_hat_transpose = DenseMatrix.OfArray(new double[,]
            {
                {-1,  1,  0,  0,  0,  1},
                {-1, -1,  0,  0,  0, -1},
                { 1,  1,  0,  0,  0, -1},
                { 1, -1,  0,  0,  0,  1},
                { 0,  0, 1,  1, -1,  0}, // NOTE: var fel här innan från i höstas!! 
                { 0,  0, 1, -1, -1,  0},
                { 0,  0, 1,  1,  1,  0},
                { 0,  0, 1, -1,  1,  0}, // NOTE: var fel här innan från i höstas!! TODO: dubbel kolla
            });
        }
        
        public float[] SITL(float[] dofInput)
        {
            float[] u = RCInput(dofInput);
            float[] mHat = MotorCommand(u);
            float[] mPwms = RCOutput(mHat);

            return mPwms;

        }
        
        private float[] RCInput(float[] dofInput)
        {
            // surge, sway, heave, roll, pitch, and yaw
            float[] u = new float[6];
            
            for (int i = 0; i < dofInput.Length; i++) // TODO: dofInput.Length == 6
            {
                u[i] = 2.0f * (dofInput[i] - 1100f) / 800f - 1.0f;
            }
            
            return u;
        }
        private float[] MotorCommand(float[] u)
        {
            int i;                                  // general purpose counter
            // TODO: double check the structure of these:
            // TODO: also make sure it is integrated from the ros side aswell
            float   roll_thrust     = u[0];         // roll thrust input value, +/- 1.0
            float   pitch_thrust    = u[1];         // pitch thrust input value, +/- 1.0
            float   yaw_thrust      = u[2];         // yaw thrust input value, +/- 1.0
            float   throttle_thrust = u[3];         // throttle thrust input value, +/- 1.0
            float   forward_thrust  = u[4];         // forward thrust input value, +/- 1.0
            float   lateral_thrust  = u[5];         // lateral thrust input value, +/- 1.0
            
            // Get rcin
            /*
            roll_thrust = (_roll_in + _roll_in_ff);
            pitch_thrust = (_pitch_in + _pitch_in_ff);
            yaw_thrust = (_yaw_in + _yaw_in_ff);
            throttle_thrust = get_throttle_bidirectional();
            forward_thrust = _forward_in;
            lateral_thrust = _lateral_in;
            */
            
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
                    rpt_out[i] = roll_thrust * (float) T_hat_transpose[i, 3] +
                                 pitch_thrust * (float) T_hat_transpose[i, 4] +
                                 throttle_thrust * (float) T_hat_transpose[i, 5];
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
                    yfl_out[i] = yaw_thrust * (float) T_hat_transpose[i, 1] +
                                 forward_thrust * (float) T_hat_transpose[i, 2] +
                                 lateral_thrust * (float) T_hat_transpose[i, 6];
                    if (math.abs(yfl_out[i]) > yfl_max) {
                        yfl_max = math.abs(yfl_out[i]);
                    }
                //}
            }
            
            // Calculate final output for each motor and normalize if necessary
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                //if (motor_enabled[i]) { // NOTE: this is for arm function in ardusub. not implemented in this sim
                    //_thrust_rpyt_out[i] = Math.Clamp(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
                    _thrust_rpyt_out[i] = Math.Clamp((rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
                    _thrust_rpyt_out_scaled[i] = 1500 + 400*_thrust_rpyt_out[i];
                //}
            }

            return _thrust_rpyt_out;
        }

        private float[] RCOutput(float[] pwms)
        {
            /*
             * [-1.0, 1.0]
             */
            //brovPhysics.SendPwmToThrusters(pwms);
            // for-loop: u = 1500 + 400*_thrust_rpyt_out[i];
            float[] u = pwms;
            return u;
        }
    }
}