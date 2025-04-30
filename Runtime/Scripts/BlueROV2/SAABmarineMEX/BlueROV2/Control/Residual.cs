using UnityEngine;
using Unity.MLAgents.Actuators;

public class Residual : MonoBehaviour
{
    PythonModelClient client;
    Rigidbody rb;

    // Keep last step’s velocities so we can compute acceleration
    Vector3 lastLinearVel;
    Vector3 lastAngularVel;

    // The ML-Agents actions from OnActionReceived
    ActionBuffers lastActions;

    void Awake()
    {
        client = FindObjectOfType<PythonModelClient>();
        rb     = GetComponent<Rigidbody>();

        // initialize
        lastLinearVel  = rb.velocity;
        lastAngularVel = rb.angularVelocity;
    }

    /// <summary>
    /// Called from your AgentController.OnActionReceived
    /// to hand off the raw actions for this physics tick.
    /// </summary>
    public void RecordActions(ActionBuffers actions)
    {
        lastActions = actions;
    }

    void FixedUpdate()
    {
        // 1) build feature vector
        float[] features = GatherFeatures();

        // 2) send to Python, get residual accelerations
        float[] residuals = client.QueryResiduals(features);
        if (residuals != null)
        {
            // 3) apply them
            ApplyResiduals(residuals);
        }
    }

    float[] GatherFeatures()
    {
        float dt = Time.fixedDeltaTime;

        // current sim velocities
        Vector3 linVel = rb.velocity;
        Vector3 angVel = rb.angularVelocity;

        // compute sim accelerations
        Vector3 linAcc = (linVel  - lastLinearVel)  / dt;
        Vector3 angAcc = (angVel  - lastAngularVel) / dt;

        // store for next step
        lastLinearVel  = linVel;
        lastAngularVel = angVel;

        // control inputs from ML-Agents
        var ctrl = lastActions.ContinuousActions; // e.g. 2- or 3-dim

        // --- assemble into [ sim_vel(6), sim_acc(6), control(N) ] ---
        int N = 6 + 6 + ctrl.Length;
        float[] feat = new float[N];
        // sim_vel: X,Y,Z,   WX,WY,WZ
        feat[0] = linVel.x; feat[1] = linVel.y; feat[2] = linVel.z;
        feat[3] = angVel.x; feat[4] = angVel.y; feat[5] = angVel.z;
        // sim_acc: X,Y,Z,   WX,WY,WZ
        feat[6]  = linAcc.x; feat[7]  = linAcc.y; feat[8]  = linAcc.z;
        feat[9]  = angAcc.x; feat[10] = angAcc.y; feat[11] = angAcc.z;
        // controls
        for (int i = 0; i < ctrl.Length; i++)
            feat[12 + i] = ctrl[i];

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
