using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using VehicleComponents.Actuators;


namespace DefaultNamespace.BlueROV2.Physics
{
    public class BrovDynamics
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

        public ThrusterT200 PropTopBackRight;
        public ThrusterT200 PropTopFrontRight;
        public ThrusterT200 PropTopBackLeft;
        public ThrusterT200 PropTopFrontLeft;
        public ThrusterT200 PropBotBackRight;
        public ThrusterT200 PropBotFrontRight;
        public ThrusterT200 PropBotBackLeft;
        public ThrusterT200 PropBotFrontLeft;
        private List<ThrusterT200> thrusters;
        
        public GameObject map;
        
        
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
        // Ardusub T matrix
        private Matrix<double> T_hat= DenseMatrix.OfArray(new double[,]
        {
            {-1,  1,  0,  0,  0,  1},
            {-1, -1,  0,  0,  0, -1},
            { 1,  1,  0,  0,  0, -1},
            { 1, -1,  0,  0,  0,  1},
            { 0,  0, 1,  1, -1,  0}, // NOTE: var fel här innan från i höstas!! 
            { 0,  0, 1, -1, -1,  0},
            { 0,  0, 1,  1,  1,  0},
            { 0,  0, 1, -1,  1,  0}, // NOTE: var fel här innan från i höstas!! TODO: dubbel kolla
                
        }).Transpose();
        
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

        void Start()
        {
            map = GameObject.Find("map"); // map frame as in ros
            
            
            // Get all propeller components
            PropTopBackRight = GameObject.Find("PropTopBackRight").GetComponent<ThrusterT200>();
            PropTopFrontRight = GameObject.Find("PropTopFrontRight").GetComponent<ThrusterT200>();
            PropTopBackLeft = GameObject.Find("PropTopBackLeft").GetComponent<ThrusterT200>();
            PropTopFrontLeft = GameObject.Find("PropTopFrontLeft").GetComponent<ThrusterT200>();
            PropBotBackRight = GameObject.Find("PropBotBackRight").GetComponent<ThrusterT200>();
            PropBotFrontRight = GameObject.Find("PropBotFrontRight").GetComponent<ThrusterT200>();
            PropBotBackLeft = GameObject.Find("PropBotBackLeft").GetComponent<ThrusterT200>();
            PropBotFrontLeft = GameObject.Find("PropBotFrontLeft").GetComponent<ThrusterT200>();
            
            thrusters = new List<ThrusterT200>
            {
                PropTopBackRight,
                PropTopFrontRight,
                PropTopBackLeft,
                PropTopFrontLeft,
                PropBotBackRight,
                PropBotFrontRight,
                PropBotBackLeft,
                PropBotFrontLeft
            };
            
            
            // Get all propeller articulation bodies
            prop_top_back_right = GameObject.Find("prop_top_back_right_link").GetComponent<ArticulationBody>();
            prop_top_front_right = GameObject.Find("prop_top_front_right_link").GetComponent<ArticulationBody>();
            prop_top_back_left = GameObject.Find("prop_top_back_left_link").GetComponent<ArticulationBody>();
            prop_top_front_left = GameObject.Find("prop_top_front_left_link").GetComponent<ArticulationBody>();
            prop_bot_back_right = GameObject.Find("prop_bot_back_right_link").GetComponent<ArticulationBody>();
            prop_bot_front_right = GameObject.Find("prop_bot_front_right_link").GetComponent<ArticulationBody>();
            prop_bot_back_left = GameObject.Find("prop_bot_back_left_link").GetComponent<ArticulationBody>();
            prop_bot_front_left = GameObject.Find("prop_bot_front_left_link").GetComponent<ArticulationBody>();
            
            // Matrices
            
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
        }

        void FixedUpdate()
        {
            // 1. Get state
            //Vector<float> poseVec = GetPoseNED();
            //float x   = poseVec[0], y     = poseVec[1], z   = poseVec[2];
            //float phi = poseVec[3], theta = poseVec[4], tau = poseVec[5];
            Vector<double> rotVec = GetRotNED();
            Vector<double> velVec = GetVelsNED();
            float u = (float) velVec[0], v = (float) velVec[1], w = (float) velVec[2];
            float p = (float) velVec[3], q = (float) velVec[4], r = (float) velVec[5];
            
            
            // 2. Calculate matrices dependent on state
            Matrix<double> C = CalculateCMatrix(u, v, w, p, q, r);
            CalculateBoancy();
            Matrix<double> D_of_vel = CalculateDMatrix(u, v, w, p, q, r);
            
            
            // 3. Calculate fossen forces
            Vector<double> tauCoriolis, tauDamping, tauRestoring, tauAddedInertia;
            CalculateFossenForces(C, D_of_vel, rotVec, velVec, out tauCoriolis, out tauDamping, out tauRestoring, out tauAddedInertia);
            
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
            dampingForce    = NED.ConvertToRUF(dampingForce);
            dampingTorque   = FRD.ConvertAngularVelocityToRUF(dampingTorque);
            coriolisForce   = NED.ConvertToRUF(coriolisForce);
            coriolisTorque  = FRD.ConvertAngularVelocityToRUF(coriolisTorque);
            restoringForce  = NED.ConvertToRUF(restoringForce);
            restoringTorque = FRD.ConvertAngularVelocityToRUF(restoringTorque);
            inputForce      = NED.ConvertToRUF(inputForce);
            inputTorque     = FRD.ConvertAngularVelocityToRUF(inputTorque);
            addedForce      = NED.ConvertToRUF(addedForce);
            addedTorque     = FRD.ConvertAngularVelocityToRUF(addedTorque);
            
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
            
            // Reset input forces every fixed update
            inputForce = Vector3.zero;
            inputTorque = Vector3.zero;
        }
        
        private void CalculateFossenForces( Matrix<double> C,
                                            Matrix<double> D_of_vel,
                                            Vector<double> rotVec,
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
            float phi = (float) rotVec[0], theta = (float) rotVec[1], psi = (float) rotVec[2];
            
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

        public void SimulateFromThrusters(float[] mPwms)
        {
            // Get thruster forces
            double[] thrusterForces = new double[thrusters.Count];
            for (int i = 0; i < thrusters.Count; i++)
            {
                thrusterForces[i] = thrusters[i].PwmToForce(mPwms[i]);
            }
            var forceVector = Vector<double>.Build.Dense(thrusterForces);
            
            // Convert to input tau acting on body
            var bodyTau = T_hat * forceVector;
            //inputForce = bodyTau.SubVector(0, 3);   // First 3 elements: indices 0,1,2
            inputForce = new Vector3((float) bodyTau[0], (float) bodyTau[1], (float) bodyTau[2]);
            //inputTorque = bodyTau.SubVector(3, 3);  // Last 3 elements: indices 3,4,5
            inputTorque = new Vector3((float) bodyTau[3], (float) bodyTau[4], (float) bodyTau[5]);
        }

        public void SimulateFromMaxTau(float[] dofPwms)
        {
            
        }
        public Vector<float> GetPoseNED()
        {
            // Get pos and rot vectors
            Vector<float> pos = GetPosNED(); // Assuming this returns Vector<float>
            Quaternion q = GetQuaternionNED(); // Assuming this returns Vector<float>
            Vector<float> rot = Vector<float>.Build.Dense(new float[] { q.x, q.y, q.z, q.w });

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
        public Vector<float> GetPosNED()
        {
            Vector3 localPosition = map.transform.InverseTransformPoint(mainBody.transform.position);
            var xyz = localPosition.To<NED>().ToDense();
            float x = (float) xyz[0];
            float y = (float) xyz[1];
            float z = (float) xyz[2];
            return Vector<float>.Build.DenseOfArray(new float[] { x, y, z });
        }

        public Vector<double> GetRotNED()
        {
            Vector3 localRotation = map.transform.InverseTransformPoint(mainBody.transform.rotation.eulerAngles);
            var phiThetaPsi = localRotation.To<NED>().ToDense();
            
            // TODO: double check this that this is what mr fossen wants
            float phi = (float) (Mathf.Deg2Rad * phiThetaPsi[0]); 
            float theta = (float) (Mathf.Deg2Rad * phiThetaPsi[1]);
            float psi = (float) (Mathf.Deg2Rad * phiThetaPsi[2]);
            
            return Vector<double>.Build.DenseOfArray(new double[] { phi, theta, psi });
        }
        
        public Quaternion GetQuaternionNED()
        {
            // TODO: test
            // Get orientation in Unity's ENU frame
            //Quaternion q_enu_local = map.transform.InverseTransformPoint(mainBody.transform.rotation);
            Quaternion localRotation = Quaternion.Inverse(map.transform.rotation) * mainBody.transform.rotation;
            // Convert quaternion from ENU to NED
            //Quaternion q_ned_local = conversionQuaternion * q_enu * Quaternion.Inverse(conversionQuaternion);

            return localRotation;
        }
        
        public Vector<double> GetVelsNED()
        {
            // Get linear and angular velocities
            Vector<double> linVels = GetLinVelsNED(); // Assuming this returns Vector<float>
            Vector<double> angVels = GetAngVelsNED(); // Assuming this returns Vector<float>
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
        
        // Velocities
        public Vector<double> GetLinVelsNED()
        {
            var inverseTransformDirection = mainBody.transform.InverseTransformDirection(mainBody.linearVelocity); // Local frame vel
            var uvw = inverseTransformDirection.To<NED>().ToDense();
            float u = (float) uvw[0];
            float v = (float) uvw[1];
            float w = (float) uvw[2];
            return Vector<double>.Build.DenseOfArray(new double[] { u, v, w });
        }

        public Vector<double> GetAngVelsNED()
        {
            var transformAngularVelocity = mainBody.transform.InverseTransformDirection(mainBody.angularVelocity); // Local frame angular vel (gives negative velocities)
            // Convert angles, angular velocities and velocities to OSBS coordinate system
            var pqr = FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity).ToDense(); // FRD is same as NED for ANGLES ONLY
            float p = (float) pqr[0];
            float q = (float) pqr[1];
            float r = (float) pqr[2];
            return Vector<double>.Build.DenseOfArray(new double[] { p, q, r });
        }

        
        
        public void SetZeroVels()
        {
            mainBody.linearVelocity = Vector3.zero;
            mainBody.angularVelocity = Vector3.zero;
        }

        public void SetPose(Vector3 localPosition, Quaternion localRotation)
        {
            // Convert to world-space using the parent's transform
            Transform parentTransform = transform.parent;
            Vector3 worldPosition = parentTransform.TransformPoint(localPosition);
            //Quaternion worldRotation = parentTransform.rotation * localRotation;
            mainBody.TeleportRoot(worldPosition, localRotation);
        }
        
        public Vector3 GetForwardUnitVec() { return mainBody.transform.forward; }
        public float GetHeight() { return mainBody.transform.position.y; }
        
        
        
        
    }
}