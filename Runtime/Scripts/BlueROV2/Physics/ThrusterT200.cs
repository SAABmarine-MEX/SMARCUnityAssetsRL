using UnityEngine;
using Unity.Mathematics;


namespace DefaultNamespace.BlueROV2.Physics
{
    public class ThrusterT200 : MonoBehaviour
    {
        public double PwmToForce2(double pwm)
        {
            // pwm: This one is nomrilizaed [-1, 1]
            // TODO: testa denna igen efter tagit bort this.pwm
            double force = -140.3*math.pow(pwm,9)+389.9*math.pow(pwm,7)-404.1*math.pow(pwm,5)+176.0*math.pow(pwm,3)+8.9*pwm;
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
    }
}