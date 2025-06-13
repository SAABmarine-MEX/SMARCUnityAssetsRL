using UnityEngine;
using Unity.MLAgents.Actuators;
using MathNet.Numerics.LinearAlgebra;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using NUnit.Framework.Constraints;
using System;

namespace DefaultNamespace.BlueROV2.Control
{ 
    public class ResidualPrepper : MonoBehaviour
    {
        PythonModelHttpClient client;
        
        // Keep last step’s velocities so we can compute acceleration
        Vector3 lastLinearVel;
        Vector3 lastAngularVel;

        private Vector3 linVel;
        private Vector3 angVel;

        // The ML-Agents actions from OnActionReceived
        float[] lastActions;

        private float[] residuals;
        
        // To measure time
        Stopwatch sw = new Stopwatch();
        
        void Awake()
        {
            client = FindObjectOfType<PythonModelHttpClient>();
        }

        public void SetActions(float[] dofControl)
        {
            float[] actions = new float[dofControl.Length];
            for (int i = 0; i < dofControl.Length; i++)
            {
                actions[i] = Scale(dofControl[i]);
            }
            lastActions = actions;
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
        
        public float[] GatherFeatures()
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
        

    }
}