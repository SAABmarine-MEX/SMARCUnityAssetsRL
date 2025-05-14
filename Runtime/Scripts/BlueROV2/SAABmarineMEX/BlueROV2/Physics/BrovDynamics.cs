using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
//using VehicleComponents.Actuators;

/*
 * The article: An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot, refered to as (OSBS) in the code,
 * is where the bluerov2 parameter values are taken from. More details: https://vbn.aau.dk/ws/portalfiles/portal/505520780/jmse_10_01898.pdf
 */


namespace DefaultNamespace.BlueROV2.Physics
{
    public class BrovDynamics : MonoBehaviour
    {
        // Flags
        public bool allowFixedUpdate = true;
        
        
        // Components
        public ArticulationBody mainBody;
        
        /*
        public ArticulationBody prop_top_back_right;
        public ArticulationBody prop_top_front_right;
        public ArticulationBody prop_top_back_left;
        public ArticulationBody prop_top_front_left;
        public ArticulationBody prop_bot_back_right;
        public ArticulationBody prop_bot_front_right;
        public ArticulationBody prop_bot_back_left;
        public ArticulationBody prop_bot_front_left;
        */
        
        public ThrusterT200 PropTopBackRight;
        public ThrusterT200 PropTopFrontRight;
        public ThrusterT200 PropTopBackLeft;
        public ThrusterT200 PropTopFrontLeft;
        public ThrusterT200 PropBotBackRight;
        public ThrusterT200 PropBotFrontRight;
        public ThrusterT200 PropBotBackLeft;
        public ThrusterT200 PropBotFrontLeft;
        private List<ThrusterT200> thrusters;
        
        private GameObject map;
        
        
        // BlueROV2 physics variables
        private double m = 14.0; //mass kg
        private double W = 0; //weight N
        private double B = 0; // bouyancy N
        double g = 9.82; // gravity m/s²
        double rho = 1000; // water density [kg/m^3]
        //double nabla = 0.0134; // volume of BlueRoV [m^3]
        double nabla = 0.0141; // volume of BlueRoV [m^3]. "floating" still at 14.1 ish


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

        // Matrices
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
        private Vector2[] minMaxes;
        // These will act on the articulated body of the Brov
        Vector3 inputForce = Vector3.zero;
        Vector3 inputTorque = Vector3.zero;

        
        public void Setup(GameObject mapframe)
        {
            map = mapframe;
        }

        void Start()
        {
            if (map == null)
            {
                Debug.LogError("map not set");
            }
            
            
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
            
            
            // min max ranges for each dof, from OSBS. Ordered as OverrideRCIn
            minMaxes = new Vector2[nInput];
            minMaxes[0] = new Vector2(-14f, 14f); // roll
            minMaxes[1] = new Vector2(-14f, 14f); // pitch
            minMaxes[2] = new Vector2(-122f, 122f); // z
            minMaxes[3] = new Vector2(-14f, 14f); // yaw
            minMaxes[4] = new Vector2(-85f, 85f); // x
            minMaxes[5] = new Vector2(-85f, 85f); // y
            
            /*
            // from https://www.mdpi.com/2076-3417/14/17/7453#FD5-applsci-14-07453
            minMaxes[0] = new Vector2(-141.29f, 141.29f); // x
            minMaxes[1] = new Vector2(-141.29f, 141.29f); // y
            minMaxes[2] = new Vector2(-199.81f, 199.81f); // z
            minMaxes[3] = new Vector2(-43.56f, 43.56f); // roll
            minMaxes[4] = new Vector2(-23.98f, 23.98f); // pitch
            minMaxes[5] = new Vector2(-37.72f, 37.72f); // yaw
            */
            
            
            // Get thrusters
            PropTopBackRight  = transform.Find("Actuators/PropTopBackRight").GetComponent<ThrusterT200>();
            PropTopFrontRight = transform.Find("Actuators/PropTopFrontRight").GetComponent<ThrusterT200>();
            PropTopBackLeft   = transform.Find("Actuators/PropTopBackLeft").GetComponent<ThrusterT200>();
            PropTopFrontLeft  = transform.Find("Actuators/PropTopFrontLeft").GetComponent<ThrusterT200>();
            PropBotBackRight  = transform.Find("Actuators/PropBotBackRight").GetComponent<ThrusterT200>();
            PropBotFrontRight = transform.Find("Actuators/PropBotFrontRight").GetComponent<ThrusterT200>();
            PropBotBackLeft   = transform.Find("Actuators/PropBotBackLeft").GetComponent<ThrusterT200>();
            PropBotFrontLeft  = transform.Find("Actuators/PropBotFrontLeft").GetComponent<ThrusterT200>();
            
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
            
            
            /*
            // Get all propeller articulation bodies
            prop_top_back_right = GameObject.Find("odom/base_link/prop_top_back_right_link").GetComponent<ArticulationBody>();
            prop_top_front_right = GameObject.Find("odom/base_link/prop_top_front_right_link").GetComponent<ArticulationBody>();
            prop_top_back_left = GameObject.Find("odom/base_link/prop_top_back_left_link").GetComponent<ArticulationBody>();
            prop_top_front_left = GameObject.Find("odom/base_link/prop_top_front_left_link").GetComponent<ArticulationBody>();
            prop_bot_back_right = GameObject.Find("odom/base_link/prop_bot_back_right_link").GetComponent<ArticulationBody>();
            prop_bot_front_right = GameObject.Find("odom/base_link/prop_bot_front_right_link").GetComponent<ArticulationBody>();
            prop_bot_back_left = GameObject.Find("odom/base_link/prop_bot_back_left_link").GetComponent<ArticulationBody>();
            prop_bot_front_left = GameObject.Find("odom/base_link/prop_bot_front_left_link").GetComponent<ArticulationBody>();
            */
            
            // Get mass from unity + one time calculations
            //m = mainBody.mass; // hk-demo mass: 14.57kg
            I_x = mainBody.inertiaTensor.x;
            I_y = mainBody.inertiaTensor.z;
            I_z = mainBody.inertiaTensor.y; // y z switch. Unity to NED coordinates
            W = m * g; // weight
            B = rho*g*nabla; // The buoyancy in [N] given by OSBS
        }
        
        void FixedUpdate()
        {
            if (!allowFixedUpdate) return;

            UpdateDynamics();
        }

        
        // Dynamics methods
        public void UpdateDynamics()
        {
            // 1. Get state
            Vector<double> rotVec = GetRotNED();
            Vector<float> velVecFloat = GetVelsNED();
            Vector<double> velVec = Vector<double>.Build.Dense(velVecFloat.Count, i => (double)velVecFloat[i]);
            
            
            // 2. Calculate matrices dependent on state
            Matrix<double> C = CalculateCMatrix(velVec);
            CalculateBouyancy();
            Matrix<double> D_of_vel = CalculateDMatrix(velVec);
            
            
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
            // Apply forces and torques
            mainBody.AddRelativeForce(-dampingForce - coriolisForce - restoringForce - addedForce + inputForce);
            mainBody.AddRelativeTorque(-dampingTorque - coriolisTorque - restoringTorque - addedTorque + inputTorque);

            /*
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
            */
            
            
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
            
            // TODO: J och D verkar dålig från pappret, l 
        }
        
        private Matrix<double> CalculateCMatrix(Vector<double> velVec)
        {
            float u = (float) velVec[0], v = (float) velVec[1], w = (float) velVec[2];
            float p = (float) velVec[3], q = (float) velVec[4], r = (float) velVec[5];
            
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
        
        private Matrix<double> CalculateDMatrix(Vector<double> velVec)
        {
            float u = (float) velVec[0], v = (float) velVec[1], w = (float) velVec[2];
            float p = (float) velVec[3], q = (float) velVec[4], r = (float) velVec[5];
            
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
        
        private void CalculateBouyancy()
        {
            var worldPos = mainBody.transform.position;
            if (worldPos.y >= 0)
            {
                B = 0;
            }
            else
            {
                B = rho*g*nabla;
            }
        }
        
        public float[] CalculateResidualTau(float[] aRes)
        {
            // F = m * acc_lin
            Vector3 forceRes = new Vector3(aRes[0], aRes[1], aRes[2]) * (float) m;
            // M = I * acc_ang
            Vector3 torqueRes = new Vector3(aRes[3] * (float) I_x, aRes[4] * (float) I_y, aRes[5] * (float) I_z);
            
            float[] bodyResTau = new float[]
            {
                forceRes[0],
                forceRes[1],
                forceRes[2],
                torqueRes[0],
                torqueRes[1],
                torqueRes[2],
            };
            
            return bodyResTau;
        }
        
        
        // Getters
        public float[] GetBodyTauFromThrusters(float[] mPwms)
        {
            // Get thruster forces
            double[] thrusterForces = new double[thrusters.Count];
            for (int i = 0; i < thrusters.Count; i++)
            {
                thrusterForces[i] = thrusters[i].PwmToForce(mPwms[i]);
            }
            var forceVector = Vector<double>.Build.Dense(thrusterForces);
            
            // Convert to input tau acting on body
            var bodyTau = T * forceVector;
            //inputForce = bodyTau.SubVector(0, 3);   // First 3 elements: indices 0,1,2
            //Vector3 inputForce2 = new Vector3((float) bodyTau[0], (float) bodyTau[1], (float) bodyTau[2]);
            float[] bodyTauFloat = new float[]
            {
                (float) bodyTau[0],
                (float) bodyTau[1],
                (float) bodyTau[2],
                (float) bodyTau[3],
                (float) bodyTau[4],
                (float) bodyTau[5],
            };
            //inputTorque = bodyTau.SubVector(3, 3);  // Last 3 elements: indices 3,4,5
            //Vector3 inputTorque2 = new Vector3((float) bodyTau[3], (float) bodyTau[4], (float) bodyTau[5]);
            return bodyTauFloat;
        }
        public float[] GetBodyTauFromMaxTau(float[] dofPwms)
        {
            Vector<float> dofTau = GetScaledActions(dofPwms);
            print("dof tau control: ");
            print(dofTau[0] + " " + dofTau[1] + " " + dofTau[2] + " " + dofTau[3] + ", " + dofTau[4] + ", " + dofTau[5]);
            
            return dofTau.ToArray();
        }
        public Vector<float> GetScaledActions(float[] actionsNorm) // OverrideRCIn order
        {
            Vector<float> actionsScaled = Vector<float>.Build.Dense(6, 0f);
            for (int i = 0; i < nInput; i++)
            {
                actionsScaled[i] = ((actionsNorm[i] + 1f) / 2f) * (minMaxes[i].y - minMaxes[i].x) + minMaxes[i].x;
            }
            return actionsScaled;
        }
        
        // State getters
        public Vector<float> GetPoseNED()
        {
            // Get pos and orientation vectors
            Vector3 pos = GetPosNED();
            Quaternion q = GetQuaternionNED();

            Vector<float> state = Vector<float>.Build.DenseOfArray(new float[]
            {
                pos.x, pos.y, pos.z,
                q.x, q.y, q.z, q.w
            });

            return state;
        }
        public Vector3 GetPosNED()
        {
            map.transform.InverseTransformPoint(mainBody.transform.position);
            Vector3 localPosition = map.transform.InverseTransformPoint(mainBody.transform.position);
            var xyz = localPosition.To<NED>().ToDense();
            return new Vector3((float) xyz[0], (float) xyz[1], (float) xyz[2]);
        }
        public Vector<double> GetRotNED() // [rad]
        {
            var world_rot = mainBody.transform.rotation.eulerAngles; 

            var phiThetaTau = FRD.ConvertAngularVelocityFromRUF(world_rot).ToDense();
            float phi   = (float) (Mathf.Deg2Rad * phiThetaTau[0]);
            float theta = (float) (Mathf.Deg2Rad * phiThetaTau[1]);
            float psi   = (float) (Mathf.Deg2Rad * phiThetaTau[2]);

            return Vector<double>.Build.DenseOfArray(new double[] { phi, theta, psi });
        }
        public Quaternion GetQuaternionNED()
        {
            // Get orientation in Unity's ENU frame
            Quaternion localRotation = Quaternion.Inverse(map.transform.rotation) * mainBody.transform.rotation;
            Quaternion<NED> rotationNed = localRotation.To<NED>();
            Quaternion rot = rotationNed.ToUnityQuaternion();

            return rot;
        }
        // Velocities
        public Vector<float> GetVelsNED()
        {
            Vector3 linVels = GetLinVelsNED();
            Vector3 angVels = GetAngVelsNED();
            
            Vector<float> stateVels = Vector<float>.Build.DenseOfArray(new float[]
            {
                linVels.x, linVels.y, linVels.z, 
                angVels.x, angVels.y, angVels.z,
            });
            
            return stateVels;
        }
        public Vector3 GetLinVelsNED()
        {
            var inverseTransformDirection = mainBody.transform.InverseTransformDirection(mainBody.linearVelocity); // Local frame vel
            var uvw = inverseTransformDirection.To<NED>().ToDense();
            float u = (float) uvw[0];
            float v = (float) uvw[1];
            float w = (float) uvw[2];
            return new Vector3(u, v, w);
        }
        public Vector3 GetAngVelsNED()
        {
            var transformAngularVelocity = mainBody.transform.InverseTransformDirection(mainBody.angularVelocity); // Local frame angular vel (gives negative velocities)
            // Convert angles, angular velocities and velocities to OSBS coordinate system
            var pqr = FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity).ToDense(); // FRD is same as NED for ANGLES ONLY
            float p = (float) pqr[0];
            float q = (float) pqr[1];
            float r = (float) pqr[2];
            return new Vector3(p, q, r);
        }
        
        public Vector3 GetForwardUnitVec() { return mainBody.transform.forward; }
        public float GetHeight() { return mainBody.transform.position.y; }

        
        // Setters
        public void SetInputTauNED(float[] bodyTau)
        {
            inputForce = new Vector3(bodyTau[0], bodyTau[1], bodyTau[2]);
            inputTorque = new Vector3(bodyTau[3], bodyTau[4], bodyTau[5]);
        }
        public void AddInputTauNED(float[] bodyTau)
        {
            for (int i = 0; i < 3; ++i)
            {
                inputForce[i] += bodyTau[i];
                inputTorque[i] += bodyTau[i+3];

            }
        }
        public void SetZeroVels()
        {
            mainBody.linearVelocity = Vector3.zero;
            mainBody.angularVelocity = Vector3.zero;
        }
        public void SetPose(Vector3 localPosition, Quaternion localRotation) //TODO: should make one with NED
        {
            // Convert to world-space using the parent's transform
            Transform parentTransform = transform.parent;
            Vector3 worldPosition = parentTransform.TransformPoint(localPosition);
            //Quaternion worldRotation = parentTransform.rotation * localRotation;
            mainBody.TeleportRoot(worldPosition, localRotation);
        }
        public void SetPoseMap(Vector3 localPosition, Quaternion localRotation)
        {
            // Convert to world-space using the parent's transform
            Transform mapTransform = map.transform;
            Vector3 worldPosition = mapTransform.TransformPoint(localPosition);
            //Quaternion worldRotation = parentTransform.rotation * localRotation;
            mainBody.TeleportRoot(worldPosition, localRotation);
        }
        
        
        void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.layer == LayerMask.NameToLayer("Water"))
            {
                Debug.Log("Touched Water layer!");
                //B = 0;
                // Do something here
            }
        }
    }
}