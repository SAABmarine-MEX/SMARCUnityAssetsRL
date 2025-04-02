using System;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using VehicleComponents.Actuators;

/*
 * The article: An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot, refered to as (OSBS) in the code,
 * is where the bluerov2 parameter values are taken from. More details: https://vbn.aau.dk/ws/portalfiles/portal/505520780/jmse_10_01898.pdf
 */


namespace DefaultNamespace
{
    public class BrovPhysics : MonoBehaviour
    {
        public ArticulationBody mainBody;
        public ArticulationBody prop_top_back_right;
        public ArticulationBody prop_top_front_right;
        public ArticulationBody prop_top_back_left;
        public ArticulationBody prop_top_front_left;
        public ArticulationBody prop_bot_back_right;
        public ArticulationBody prop_bot_front_right;
        public ArticulationBody prop_bot_back_left;
        public ArticulationBody prop_bot_front_left;

        public Propeller PropTopBackRight;
        public Propeller PropTopFrontRight;
        public Propeller PropTopBackLeft;
        public Propeller PropTopFrontLeft;
        public Propeller PropBotBackRight;
        public Propeller PropBotFrontRight;
        public Propeller PropBotBackLeft;
        public Propeller PropBotFrontLeft;

        // Variables
        private Camera myCamera;
        private Vector3 camera_offset;
        public bool Ardusub_mode;
        public bool Arusub_prep;

        // Constants
        private double m = 0; //mass kg
        private double W = 0; //weight N
        private double B = 0; // bouyancy N
        double g = 9.82; // gravity m/s²
        double rho = 1000; // water density [kg/m^3]
        double nabla = 0.0134; // volume of BlueRoV [m^3]
        private double rpmMax = 3000;

        //Bouyancy point coordinates relative to report coordinate system
        double  x_b = 0; double y_b = 0; double z_b = -0.01;

        //Added from OSBS
        //Rotational damping (Ns/m)
        public double Xuu = 141; // #1.0
        double Yvv = 217; // #100.0
        double Zww = 190; // #100.0
        double Kpp = 1.19; // #10.0
        double Mqq = 0.47; // #100.0
        double Nrr = 1.5; // #150.0 
        //Translational damping (Ns/m)
        double Xu = 13.7;
        double Yv = 0;
        double Zw = 33;
        double Kp = 0;
        double Mq = 0.8;
        double Nr = 0;
        Matrix<double> D;
        // Added mass coefficients 
        double X_udot = 6.36; // [kg]
        double Y_vdot = 7.12; // [kg]
        double Z_wdot = 18.68; // [kg]
        double K_pdot = 0.189; // [kg*m^2]
        double M_qdot = 0.135; // [kg*m^2]
        double N_rdot = 0.222; // [kg*m^2]
        //Inertia 
        double I_x = 0.2818; // [kg*m^2], from OSBS's CAD
        double I_y = 0.245; // [kg*m^2], from OSBS's CAD
        double I_z = 0.3852; // [kg*m^2], from OSBS's CAD

        Matrix<double> M_A;
        Matrix<double> M_inv = DenseMatrix.OfDiagonalArray(new double[] // Inverted total mass matrix (rigid body + added mass)
        {
            0.0504,
            0.0485,
            0.0311,
            2.2272,
            2.7397,
            1.6892
        });

        // Define T matrix
        // Matrix<double> T = DenseMatrix.OfArray(new double[,]
        // {
        //     {-0.71, -0.71,  0.71,  0.71,  0,     0,    0,     0   },
        //     {0.71,  -0.71,  0.71, -0.71,  0,     0,    0,     0   },
        //     {0,      0,     0,     0,     1,     1,    1,     1   },
        //     {-0.06,  0.06, -0.06,  0.06,  0.22, -0.22, 0.22, -0.22},
        //     {-0.06, -0.06,  0.06,  0.06, -0.12, -0.12, 0.12,  0.12},
        //     {0.99,  -0.99, -0.99,  0.99,  0,     0,    0,     0   }
        // });
        Matrix<double> T = DenseMatrix.OfArray(new double[,]
        {
            { Math.Sqrt(2)/2,  Math.Sqrt(2)/2, -Math.Sqrt(2)/2, -Math.Sqrt(2)/2,  0,      0,       0,       0      },
            { -Math.Sqrt(2)/2, Math.Sqrt(2)/2, -Math.Sqrt(2)/2,  Math.Sqrt(2)/2,  0,      0,       0,       0      },
            { 0,               0,              0,               0,              -1,      1,       1,      -1       },
            { 0,               0,              0,               0,               0.218,  0.218,  -0.218,  -0.218   },
            { 0,                 0,              0,               0,               0.12,  -0.12,    0.12,   -0.12  },
            { -0.1888,         0.1888,         0.1888,         -0.1888,          0,      0,       0,       0       }
        });

        // Min and max force and torques acting on center of mass
        int nInput = 6;
        private Vector2[] minMaxes;// = new Vector2[nInput];
        // These will act on the articulated body of the Brov
        Vector3 inputForce = Vector3.zero;
        Vector3 inputTorque = Vector3.zero;

        // Positions
        float x,   y,     z;
        float phi, theta, tau;
        // Velocities
        float u, v, w;
        float p, q, r;

        void Start()
        {
            //Matrices
            // Dampining matrix
            D = DenseMatrix.OfDiagonalArray(new double[]
            {
                Xu,
                Yv,
                Zw,
                Kp,
                Mq,
                Nr
            });
            // Rigid body and added mass matrices
            // Matrix<double> M_RB = DenseMatrix.OfDiagonalArray(new double[] {m, m, m, I_x, I_y, I_z});
            M_A = DenseMatrix.OfDiagonalArray(new double[] {X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot});

            Debug.Log("Agent:" + gameObject.name);
            // min max ranges for each dof  TODO: double check with data sheet
            minMaxes = new Vector2[nInput];
            minMaxes[0] = new Vector2(-85f, 85f); // x
            minMaxes[1] = new Vector2(-85f, 85f); // y
            minMaxes[2] = new Vector2(-122f, 122f); // z
            minMaxes[3] = new Vector2(-14f, 14f); // roll
            minMaxes[4] = new Vector2(-14f, 14f); // pitch
            minMaxes[5] = new Vector2(-14f, 14f); // yaw
            /*
            // Get all propeller articulation bodies
            prop_top_back_right = transform.Find("odom/base_link/prop_top_back_right_link").GetComponent<ArticulationBody>();
            Debug.Log("Efter först " + gameObject.name);
	    prop_top_front_right = transform.Find("odom/base_link/prop_top_front_right_link").GetComponent<ArticulationBody>();
            prop_top_back_left = transform.Find("odom/base_link/prop_top_back_left_link").GetComponent<ArticulationBody>();
            prop_top_front_left = transform.Find("odom/base_link/prop_top_front_left_link").GetComponent<ArticulationBody>();
            prop_bot_back_right = transform.Find("odom/base_link/prop_bot_back_right_link").GetComponent<ArticulationBody>();
            prop_bot_front_right = transform.Find("odom/base_link/prop_bot_front_right_link").GetComponent<ArticulationBody>();
            prop_bot_back_left = transform.Find("odom/base_link/prop_bot_back_left_link").GetComponent<ArticulationBody>();
            prop_bot_front_left = transform.Find("odom/base_link/prop_bot_front_left_link").GetComponent<ArticulationBody>();
            Debug.Log("3 efteer prop med litet p" + gameObject.name);
	    // Get all propeller components
			
            PropTopBackRight = transform.Find("odom/base_link/prop_top_back_right_link/PropTopBackRight").GetComponent<Propeller>();
            PropTopFrontRight = transform.Find("odom/base_link/prop_top_front_right_link/PropTopFrontRight").GetComponent<Propeller>();
            PropTopBackLeft = transform.Find("odom/base_link/prop_top_back_left_link/PropTopBackLeft").GetComponent<Propeller>();
            PropTopFrontLeft = transform.Find("odom/base_link/prop_top_front_left_link/PropTopFrontLeft").GetComponent<Propeller>();
            PropBotBackRight = transform.Find("odom/base_link/prop_bot_back_right_link/PropBotBackRight").GetComponent<Propeller>();
            PropBotFrontRight = transform.Find("odom/base_link/prop_bot_front_right_link/PropBotFrontRight").GetComponent<Propeller>();
            PropBotBackLeft = transform.Find("odom/base_link/prop_bot_back_left_link/PropBotBackLeft").GetComponent<Propeller>();
            PropBotFrontLeft = transform.Find("odom/base_link/prop_bot_front_left_link/PropBotFrontLeft").GetComponent<Propeller>();
            */
            // NOTE: had probem with the above when adding multiple agents so went back to the bellow and seems to be working

            // Get all propeller components
            PropTopBackRight = GameObject.Find("PropTopBackRight").GetComponent<Propeller>();
            PropTopFrontRight = GameObject.Find("PropTopFrontRight").GetComponent<Propeller>();
            PropTopBackLeft = GameObject.Find("PropTopBackLeft").GetComponent<Propeller>();
            PropTopFrontLeft = GameObject.Find("PropTopFrontLeft").GetComponent<Propeller>();
            PropBotBackRight = GameObject.Find("PropBotBackRight").GetComponent<Propeller>();
            PropBotFrontRight = GameObject.Find("PropBotFrontRight").GetComponent<Propeller>();
            PropBotBackLeft = GameObject.Find("PropBotBackLeft").GetComponent<Propeller>();
            PropBotFrontLeft = GameObject.Find("PropBotFrontLeft").GetComponent<Propeller>();

            // Get all propeller articulation bodies
            prop_top_back_right = GameObject.Find("prop_top_back_right_link").GetComponent<ArticulationBody>();
            prop_top_front_right = GameObject.Find("prop_top_front_right_link").GetComponent<ArticulationBody>();
            prop_top_back_left = GameObject.Find("prop_top_back_left_link").GetComponent<ArticulationBody>();
            prop_top_front_left = GameObject.Find("prop_top_front_left_link").GetComponent<ArticulationBody>();
            prop_bot_back_right = GameObject.Find("prop_bot_back_right_link").GetComponent<ArticulationBody>();
            prop_bot_front_right = GameObject.Find("prop_bot_front_right_link").GetComponent<ArticulationBody>();
            prop_bot_back_left = GameObject.Find("prop_bot_back_left_link").GetComponent<ArticulationBody>();
            prop_bot_front_left = GameObject.Find("prop_bot_front_left_link").GetComponent<ArticulationBody>();

            // Get camera and set camera offset TODO: this is not needed anymore with the new 3rd person camera? test
            myCamera = Camera.main;
            camera_offset = new Vector3(0f, 2f, -4f);

            // Get mass from unity + one time calculations
            m = mainBody.mass; // hk-demo mass: 14.57kg
            I_x = mainBody.inertiaTensor.x;
            I_y = mainBody.inertiaTensor.z;
            I_z = mainBody.inertiaTensor.y; // y z switch. Unity to NED coordinates
            W = m * g; // weight
            B = rho*g*nabla; // The buoyancy in [N] given by OSBS
        }


        // Public methods
        public Vector<float> ScaleActions(Vector<float> actionsNorm)
        {
            Vector<float> actionsScaled = Vector<float>.Build.Dense(6, 0f);
            for (int i = 0; i < nInput; i++)
            {
                actionsScaled[i] = ((actionsNorm[i] + 1f) / 2f) * (minMaxes[i].y - minMaxes[i].x) + minMaxes[i].x;
            }
            return actionsScaled;
        }

        // Pose
        public Vector3 GetLocalPos()
        {
            return mainBody.transform.localPosition;
        }

        public Vector3 GetLocalPosNED()
        {
            return new Vector3(x, y, z);
        }

        public Quaternion GetLocalRot() // TODO: add method to get quaternions in ned
        {
            return transform.localRotation;
        }

        public Vector3 GetLocalRotEuler()
        {
            return transform.localRotation.eulerAngles;
        }

        public Vector<float> GetLocalRotEulerNED()
        {
            return Vector<float>.Build.DenseOfArray(new float[] 
            {
            phi * Mathf.Rad2Deg,
            theta * Mathf.Rad2Deg,
            tau * Mathf.Rad2Deg
            });
        }

        public Vector<float> GetPosNED2()
        {
            // TODO: what is the difference from doing this
            var inverseTransformDirectionPos = mainBody.transform.InverseTransformDirection(mainBody.transform.position); // Local frame pos
            // TODO: compared to this. test
            //mainBody.transform.localPosition
            var xyz = inverseTransformDirectionPos.To<NED>().ToDense(); // Transform local position to NED
            x = (float) xyz[0];
            y = (float) xyz[1];
            z = (float) xyz[2];
            return Vector<float>.Build.DenseOfArray(new float[] { x, y, z });
        }

        public Vector<float> GetRotNED2()
        {
            var world_rot = mainBody.transform.rotation.eulerAngles; 
            // TODO: is this world rot in NED? How to get local. Confusing that it says velocity. check definitions
            var phiThetaTau = FRD.ConvertAngularVelocityFromRUF(world_rot).ToDense();
            phi = (float) (Mathf.Deg2Rad * phiThetaTau[0]); 
            theta = (float) (Mathf.Deg2Rad* phiThetaTau[1]);
            tau = (float) (Mathf.Deg2Rad* phiThetaTau[2]);
            return Vector<float>.Build.DenseOfArray(new float[] { phi, theta, tau });
        }

        public Vector<float> GetStatePosNED2() // TODO: rename to GetPoseNED2() instead
        {
            // Get pos and rot vectors
            Vector<float> pos = GetPosNED2(); // Assuming this returns Vector<float>
            Vector<float> rot = GetRotNED2(); // Assuming this returns Vector<float>

            // Create a new array large enough to hold both pos and rot
            float[] combinedArray = new float[pos.Count + rot.Count];

            // Copy pos elements into the combined array using System.Array.Copy
            System.Array.Copy(pos.ToArray(), 0, combinedArray, 0, pos.Count);

            // Copy rot elements into the combined array starting after pos
            System.Array.Copy(rot.ToArray(), 0, combinedArray, pos.Count, rot.Count);

            // Create a new Vector from the combined array
            Vector<float> state = Vector<float>.Build.DenseOfArray(combinedArray);

            return state;
        }


        // Velocities
        public Vector<double> GetLinVelsNED2()
        {
            var inverseTransformDirection = mainBody.transform.InverseTransformDirection(mainBody.linearVelocity); // Local frame vel
            var uvw = inverseTransformDirection.To<NED>().ToDense();
            u = (float) uvw[0];
            v = (float) uvw[1];
            w = (float) uvw[2];
            return Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
        }

        public Vector<double> GetAngVelsNED2()
        {
            var transformAngularVelocity = mainBody.transform.InverseTransformDirection(mainBody.angularVelocity); // Local frame angular vel (gives negative velocities)
            // Convert angles, angular velocities and velocities to OSBS coordinate system
            var pqr = FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity).ToDense(); // FRD is same as NED for ANGLES ONLY
            p = (float) pqr[0];
            q = (float) pqr[1];
            r = (float) pqr[2];
            return Vector<double>.Build.DenseOfArray(new double[] { p, q, r });
        }

        public Vector<double> GetStateVelsNED2() // TODO: rename to GetVelsNED2() instead
        {
            // Get linear and angular velocities
            Vector<double> linVels = GetLinVelsNED2(); // Assuming this returns Vector<float>
            Vector<double> angVels = GetAngVelsNED2(); // Assuming this returns Vector<float>
            // Create a new array large enough to hold both linear and angular velocities
            double[] combinedArray = new double[linVels.Count + angVels.Count];

            // Copy linear velocities into the combined array using Array.Copy
            System.Array.Copy(linVels.ToArray(), 0, combinedArray, 0, linVels.Count);

            // Copy angular velocities into the combined array starting after linear velocities
            System.Array.Copy(angVels.ToArray(), 0, combinedArray, linVels.Count, angVels.Count);

            // Create a new Vector from the combined array
            Vector<double> stateVels = Vector<double>.Build.DenseOfArray(combinedArray);
            return stateVels;
        }

        public Vector<float> GetVelocity()
        {
            return Vector<float>.Build.DenseOfArray(new float[] { u, v, w, p, q, r });
        }


        public Vector3 GetForwardUnitVec() { return mainBody.transform.forward; }
        public Vector3 GetForwardUnitVecNED() 
        { 
            Vector3 unityForward = mainBody.transform.forward;
            return new Vector3(unityForward.z, unityForward.x, -unityForward.y); // TODO: use method instead 
        }


        // Interact with the body (setters)
        public void SetInput(Vector3 force, Vector3 torque)
        {
            // Input: RUF force and torque vector
            // Process: Makes these NED and add to inputForce and inputTorque
            var inputForceTemp = force.To<NED>().ToDense();
            force = new Vector3((float) inputForceTemp[0], (float) inputForceTemp[1], (float) inputForceTemp[2]);
            //inputTorque.To<NED>().ToDense()
            var inputTorqueTemp = FRD.ConvertAngularVelocityFromRUF(torque).ToDense(); // FRD is same as NED for ANGLES ONLY (Negative since inputs are right handed )
            torque = new Vector3((float) inputTorqueTemp[0], (float)inputTorqueTemp[1], (float) inputTorqueTemp[2]);
            inputForce += force;
            inputForce[2] = -inputForce[2]; // FIXME: works but looks ugly
            inputTorque += torque;
        }

        public void SetInputNED(Vector3 force, Vector3 torque)
        {
            inputForce = force;
            //inputForce[2] = -inputForce[2]; // FIXME: works but looks ugly
            inputTorque = torque;
        }

        public void SetZeroVels()
        {
            if (mainBody == null)
            {
                Debug.LogError("ArticulationBody component is missing on " + mainBody.name);
            } 
            else{ Debug.Log("VET " + mainBody.name); }
            mainBody.linearVelocity = Vector3.zero;
            mainBody.angularVelocity = Vector3.zero;
        }

        public void SetPosAndRot(Vector3 localPosition, Quaternion localRotation)
        {
            // Convert to world-space using the parent's transform
            Transform parentTransform = transform.parent;
            Vector3 worldPosition = parentTransform.TransformPoint(localPosition);
            //Quaternion worldRotation = parentTransform.rotation * localRotation;
            mainBody.TeleportRoot(worldPosition, localRotation);
        }


        // Private methods
        private void CalculateBoancy()
        {
            var worldPos = mainBody.transform.position;
            if (worldPos.y >= 0)
            {
                B = 0; // TODO: maybe make into local variable
            }
            else
            {
                B = rho*g*nabla;
            }
        }

        private void SITL() // TODO: implement SITL
        {
            // SITL:
            // if (Arusub_prep)
            // {
            //     Matrix<double> T_hat_inv = DenseMatrix.OfArray(new double[,]
            //     {
            //         { 0.25,  0.25, -0.25, -0.25,  0.0,  0.0,  0.0,  0.0 },
            //         { -0.25,  0.25, -0.25,  0.25,  0.0,  0.0,  0.0,  0.0 },
            //         { -0.0,  -0.0,  -0.0,   0.0,  -0.25, 0.25,  0.25, -0.25 },
            //         {  0.0,   0.0,   0.0,   0.0,   0.25, 0.25, -0.25, -0.25 },
            //         {  0.0,   0.0,   0.0,   0.0,   0.25, -0.25, 0.25, -0.25 },
            //         { -0.25,  0.25,  0.25, -0.25,  0.0,  0.0,  0.0,  0.0 }
            //     });
            //     
            //     ROSForces = T_hat_inv * F_vec;
            //    
            // }
            //
            //   // print("before ardusub");
            //   //           for (int i = 0; i < F_vec.Count; i++)
            //   //           {
            //   //               print(F_vec[i]);
            //   //           }
            //
            //
            //
            // if (Ardusub_mode)
            // {
            //     // print("thruster forces");
            //     // for (int i = 0; i < F_vec.Count; i++)
            //     // {
            //     //     F_vec[i] = VoltageToForce(F_vec[i]);
            //     //     print(F_vec[i]);
            //     // }
            //     Matrix<double> T_transpose = DenseMatrix.OfArray(new double[,]
            //     {
            //         {-1,  1,  0,  0,  0,  1},
            //         {-1, -1,  0,  0,  0, -1},
            //         { 1,  1,  0,  0,  0, -1},
            //         { 1, -1,  0,  0,  0,  1},
            //         { 0,  0, -1,  1, -1,  0},
            //         { 0,  0, 1, -1, -1,  0},
            //         { 0,  0, 1,  1,  1,  0},
            //         { 0,  0, -1, -1,  1,  0},
            //     });
            //     
            //     var F_vec_ardusub_unscaled = T_transpose * ROSForces;
            //     
            //     double[] yss = new double[4];
            //     double[] rph = new double[4];
            //     
            //     for (int i = 0; i < 4; i++)
            //     {
            //         yss[i] = F_vec_ardusub_unscaled[i];
            //         rph[i] = F_vec_ardusub_unscaled[i + 4];
            //     }
            //     
            //     double max_yss = 1;
            //     double max_rph = 1;
            //     
            //     for (int i = 1; i < 4; i++)
            //     {
            //         if (math.abs(yss[i]) > max_yss)
            //             max_yss = math.abs(yss[i]);
            //     
            //         if (math.abs(rph[i]) > max_rph)
            //             max_rph = math.abs(rph[i]);
            //     }
            //     
            //     for (int i = 0; i < 4; i++)
            //     {
            //         yss[i] /= max_yss;
            //         
            //         rph[i] /= max_rph;
            //     }   
            //     
            //     double[] adjust = { 1, 1, 1, 1, 1, 1, 1, 1 };
            //     
            //     Vector<double> F_vec_ardusub = Vector<double>.Build.DenseOfArray(new double[] 
            //         {
            //             (yss[0]*adjust[0]),
            //             (yss[1]*adjust[1]),
            //             (yss[2]*adjust[2]),
            //             (yss[3]*adjust[3]),
            //             (rph[0]*adjust[4]),
            //             (rph[1]*adjust[5]),
            //             (rph[2]*adjust[6]),
            //             (rph[3]*adjust[7]),
            //         }
            //     );
            //
            //     F_vec = F_vec_ardusub;
            // }

        }

        private void CalculateFossenForces( Matrix<double> C,
                                            Matrix<double> D_of_vel,
                                            Vector<double> velVec,
                                            out Vector<double> tauCoriolis,
                                            out Vector<double> tauDamping,
                                            out Vector<double> tauRestoring,
                                            out Vector<double> tauAddedInertia)
        {
            /*
             * Input: Matrices and state (velocity and rotation) for fossen
             * Fossen equations:
             * eta_dot = J(eta)*v
             * M*v_dot + C(v)*v + D(v)*v + g(eta) = tau + tau_teather
             * Output: Fossen forces
             */
            // C(v)*v: Coriolis forces
            tauCoriolis =  C * velVec;

            // D(v)*v: Dampening forces
            var v_c = 0; // Assume no ocean current. If desired to integrate it, info about it can be found in OSBS
            var vr = velVec - v_c; 
            tauDamping = D_of_vel*vr; 

            // g(eta): restoring forces
            tauRestoring = Vector<double>.Build.DenseOfArray(new double[]
            {
                (W-B)*Mathf.Sin(theta),
                -(W-B)*Mathf.Cos(theta)*Mathf.Sin(phi),
                -(W-B)*Mathf.Cos(theta)*Mathf.Cos(phi),
                y_b*B*Mathf.Cos(theta)*Mathf.Cos(phi)-z_b*B*Mathf.Cos(theta)*Mathf.Sin(phi),
                -z_b*B*Mathf.Sin(theta)-x_b*B*Mathf.Cos(theta)*Mathf.Cos(phi),
                x_b*B*Mathf.Cos(theta)*Mathf.Sin(phi)+y_b*B*Mathf.Sin(theta)
            }
            );

            // v_dot
            var reactive_force_sum = (-tauRestoring - tauDamping - tauCoriolis);
            Vector<double> input_forces_sum  = Vector<double>.Build.DenseOfArray(new double[] {inputForce[0], inputForce[1], inputForce[2], inputTorque[0], inputTorque[1], inputTorque[2] });
            var total_force_sum = reactive_force_sum + input_forces_sum; // NOTE: if want to add teather forces, it should be added here
            var vel_vec_dot = M_inv*total_force_sum;

            // m*a  TODO: ?
            tauAddedInertia = M_A * vel_vec_dot;
        }
        private Matrix<double> CalculateCMatrix(float u, float v, float w, float p, float q, float r)
        {
            // Coriollis and centripetal matrices
            Matrix<double> C_RB = DenseMatrix.OfArray(new double[,]
            {
                {0,     0,      0,      0,      m*w,    -m*v    },
                {0,     0,      0,      -m*w,   0,       m*u    },
                {0,     0,      0,      m*v,    -m*u,    0      },
                {0,     m*w,    -m*v,   0,      -I_z*r, -I_y*q  },
                {-m*w,  0,      m*u,    I_z*r,  0,       I_x*p  },
                {m*v,   -m*u,   0,      I_y*q,  -I_x*p,  0      },
            });
            Matrix<double> C_A = DenseMatrix.OfArray(new double[,]
            {
                {0,         0,          0,          0,          -Z_wdot*w,  Y_vdot*v    },
                {0,         0,          0,          Z_wdot*w,   0,          -X_udot*u   },
                {0,         0,          0,          -Y_vdot*v,  X_udot*u,   0           },
                {0,         -Z_wdot*w,  Y_vdot*v,   0,          -N_rdot*r,  M_qdot*q    },
                {Z_wdot*w,  0,          -X_udot*u,  N_rdot*r,   0,          -K_pdot*p   },
                {-Y_vdot*v, X_udot*u,   0,          -M_qdot*q,  K_pdot*p,   0           }
            });
            Matrix<double> C = C_RB + C_A;
            return C;
        }
        private Matrix<double> CalculateDMatrix(float u, float v, float w, float p, float q, float r)
        {
            // Dampening matrices
            Matrix<double> Dn = DenseMatrix.OfDiagonalArray(new double[] 
            {
                Xuu*Mathf.Abs(u),
                Yvv*Mathf.Abs(v), 
                Zww*Mathf.Abs(w),
                Kpp*Mathf.Abs(p),
                Mqq*Mathf.Abs(q), 
                Nrr*Mathf.Abs(r)
            });
            Matrix<double> D_of_vel = D + Dn;
            return D_of_vel;
        }


        void FixedUpdate()
        {
            // 1. Get state
            Vector<float> posVec = GetStatePosNED2();
            float x = posVec[0], y = posVec[1], z = posVec[2];
            float phi = posVec[3], theta = posVec[4], tau = posVec[5];
            Vector<double> velVec = GetStateVelsNED2();
            float u = (float) velVec[0], v = (float) velVec[1], w = (float) velVec[2];
            float p = (float) velVec[3], q = (float) velVec[4], r = (float) velVec[5];


            // 2. Calculate matrices dependent on state
            Matrix<double> C = CalculateCMatrix(u, v, w, p, q, r);
            CalculateBoancy();
            Matrix<double> D_of_vel = CalculateDMatrix(u, v, w, p, q, r);


            // VVV UNCOMMENT FOR FOLLOWING CAMERA VVV
            //myCamera.transform.position = camera_offset + world_pos;


            // 3. Calculate fossen forces
            Vector<double> tauCoriolis, tauDamping, tauRestoring, tauAddedInertia;
            CalculateFossenForces(C, D_of_vel, velVec, out tauCoriolis, out tauDamping, out tauRestoring, out tauAddedInertia);

            // Seperate forces and torques
            var coriolisForce = tauCoriolis.SubVector(0, 3).ToVector3();
            var coriolisTorque = tauCoriolis.SubVector(3, 3).ToVector3();
            var dampingForce = tauDamping.SubVector(0, 3).ToVector3();
            var dampingTorque = tauDamping.SubVector(3, 3).ToVector3();
            var restoringForce  = tauRestoring.SubVector(0, 3).ToVector3();
            var restoringTorque = tauRestoring.SubVector(3, 3).ToVector3();
            var addedForce = tauAddedInertia.SubVector(0, 3).ToVector3();
            var addedTorque = tauAddedInertia.SubVector(3, 3).ToVector3();

            // Convert to RUF
            dampingForce = NED.ConvertToRUF(dampingForce);
            dampingTorque = FRD.ConvertAngularVelocityToRUF(dampingTorque);
            coriolisForce = NED.ConvertToRUF(coriolisForce);
            coriolisTorque = FRD.ConvertAngularVelocityToRUF(coriolisTorque);
            restoringForce = NED.ConvertToRUF(restoringForce);
            restoringTorque = FRD.ConvertAngularVelocityToRUF(restoringTorque);
            inputForce = NED.ConvertToRUF(inputForce);
            inputTorque = FRD.ConvertAngularVelocityToRUF(inputTorque);
            addedForce = NED.ConvertToRUF(addedForce);
            addedTorque = FRD.ConvertAngularVelocityToRUF(addedTorque);

            // Add forces and torques to rigid body
            mainBody.AddRelativeForce(-dampingForce);
            mainBody.AddRelativeForce(-coriolisForce);
            mainBody.AddRelativeForce(-restoringForce);
            mainBody.AddRelativeForce(-addedForce);
            mainBody.AddRelativeForce(inputForce);
            mainBody.AddRelativeTorque(-dampingTorque);
            mainBody.AddRelativeTorque(-coriolisTorque);
            mainBody.AddRelativeTorque(-restoringTorque);
            mainBody.AddRelativeTorque(-addedTorque);
            mainBody.AddRelativeTorque(inputTorque);

            // ROS Controlls
            // Update propeller rpm's
            // Top propellers
            float rpmTopBackRight = (float)PropTopBackRight.rpm;
            float rpmTopFrontRight = (float)PropTopFrontRight.rpm;
            float rpmTopBackLeft = (float)PropTopBackLeft.rpm;
            float rpmTopFrontLeft = (float)PropTopFrontLeft.rpm;
            // Bottom propellers
            float rpmBotBackRight = (float)PropBotBackRight.rpm;
            float rpmBotFrontRight = (float)PropBotFrontRight.rpm;
            float rpmBotBackLeft = (float)PropBotBackLeft.rpm;
            float rpmBotFrontLeft = (float)PropBotFrontLeft.rpm;

            Vector<double> F_vec = Vector<double>.Build.DenseOfArray(new double[] 
            {
                rpmBotFrontRight/rpmMax,
                rpmBotFrontLeft/rpmMax,
                rpmBotBackRight/rpmMax,
                rpmBotBackLeft/rpmMax,
                rpmTopFrontRight/rpmMax,
                rpmTopFrontLeft/rpmMax,
                rpmTopBackRight/rpmMax,
                rpmTopBackLeft/rpmMax
            }
            );
            var ROSForces = T * F_vec;

            // print("arduprepped thruster forces");
            for (int i = 0; i < F_vec.Count; i++)
            {
                // if (i < 4)
                // {
                //     F_vec[i] = -(F_vec[i]);
                // }
                F_vec[i] = VoltageToForce(F_vec[i]);
                // print(F_vec[i]);
            }

            // Set RPMs for Visuals
            prop_top_back_right.SetDriveTargetVelocity(ArticulationDriveAxis.X, rpmTopBackRight);
            prop_top_front_right.SetDriveTargetVelocity(ArticulationDriveAxis.X, rpmTopFrontRight);
            prop_top_back_left.SetDriveTargetVelocity(ArticulationDriveAxis.X, rpmTopBackLeft);
            prop_top_front_left.SetDriveTargetVelocity(ArticulationDriveAxis.X, rpmTopFrontLeft);

            prop_bot_back_right.SetDriveTargetVelocity(ArticulationDriveAxis.Z, rpmBotBackRight);
            prop_bot_front_right.SetDriveTargetVelocity(ArticulationDriveAxis.Z, rpmBotFrontRight);
            prop_bot_back_left.SetDriveTargetVelocity(ArticulationDriveAxis.Z, rpmBotBackLeft);
            prop_bot_front_left.SetDriveTargetVelocity(ArticulationDriveAxis.Z, rpmBotFrontLeft);

            double VoltageToForce(double V)
            {
                double force = -140.3*math.pow(V,9)+389.9*math.pow(V,7)-404.1*math.pow(V,5)+176.0*math.pow(V,3)+8.9*V;
                return force;
            }


            // Reset input forces every fixed update
            inputForce = Vector3.zero;
            inputTorque = Vector3.zero;
        }
    }
}
