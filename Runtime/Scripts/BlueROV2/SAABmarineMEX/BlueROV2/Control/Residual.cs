using UnityEngine;
using Unity.MLAgents.Actuators;
using MathNet.Numerics.LinearAlgebra;

namespace DefaultNamespace.BlueROV2.Control
{ 
    public class Residual : MonoBehaviour
    {
        PythonModelClient client;
        Rigidbody rb;

        // Keep last step’s velocities so we can compute acceleration
        Vector3 lastLinearVel;
        Vector3 lastAngularVel;

        private Vector3 linVel;
        private Vector3 angVel;

        // The ML-Agents actions from OnActionReceived
        float[] lastActions;

        private float[] residuals;

        void Awake()
        {
            client = FindObjectOfType<PythonModelClient>();
        }

        /// <summary>
        /// Called from your AgentController.OnActionReceived
        /// to hand off the raw actions for this physics tick.
        /// </summary>
        public void SetActions(float[] dofControl)
        {
            float[] actions = new float[dofControl.Length];
            for (int i = 0; i < dofControl.Length; i++)
            {
                actions[i] = Scale(dofControl[i]);
            }
            lastActions = actions;
        }

        /*
        void FixedUpdate()
        {
            // 1) build feature vector
            float[] features = GatherFeatures();

            // 2) send to Python, get residual accelerations
            residuals = client.QueryResiduals(features);
            if (residuals != null)
            {
                // 3) apply them
                ApplyResiduals(residuals);
            }
        }
        */

        public float[] GetResiduals()
        {
            float[] features = GatherFeatures();

            // 2) send to Python, get residual accelerations
            residuals = client.QueryResiduals(features);
            return residuals;
        }
        float Scale(float value)
        {
            float originalMin = 1100;
            float originalMax = 1900;
            float targetMin = -1.0f;
            float targetMax = 1.0f;

            return (value - originalMin) / (originalMax - originalMin) * (targetMax - targetMin) + targetMin;
        }

        public void SetVelsNED(Vector<float> vels)
        {
            linVel = new Vector3(vels[0], vels[1], vels[2]);
            angVel = new Vector3(vels[3], vels[4], vels[5]);
        }

        float[] GatherFeatures()
        {
            float dt = Time.fixedDeltaTime;

            // compute sim accelerations
            Vector3 linAcc = (linVel  - lastLinearVel)  / dt;
            Vector3 angAcc = (angVel  - lastAngularVel) / dt;

            // store for next step
            lastLinearVel  = linVel;
            lastAngularVel = angVel;

            // control inputs from ML-Agents

            // --- assemble into [ sim_vel(6), sim_acc(6), control(N) ] ---
            int N = 6 + 6 + lastActions.Length;
            float[] feat = new float[N];
            // sim_vel: X,Y,Z,   WX,WY,WZ
            feat[0] = linVel.x; feat[1] = linVel.y; feat[2] = linVel.z;
            feat[3] = angVel.x; feat[4] = angVel.y; feat[5] = angVel.z;
            // sim_acc: X,Y,Z,   WX,WY,WZ
            feat[6]  = linAcc.x; feat[7]  = linAcc.y; feat[8]  = linAcc.z;
            feat[9]  = angAcc.x; feat[10] = angAcc.y; feat[11] = angAcc.z;
            // controls
            for (int i = 0; i < lastActions.Length; i++)
                feat[12 + i] = lastActions[i];

            return feat;
        }

        void ApplyResiduals(float[] residuals)
        {
            // residuals = Δacc [X,Y,Z, WX,WY,WZ]
            // F = m * a,   τ ≈ I * α   (we’ll use mass for torque scale too)
            float m = rb.mass;
            Vector3 linRes = new Vector3(residuals[0], residuals[1], residuals[2]) * m;
            Vector3 angRes = new Vector3(residuals[3], residuals[4], residuals[5]) * m;

            rb.AddForce(linRes, ForceMode.Force);
            rb.AddTorque(angRes, ForceMode.Force);
        }
    }
}