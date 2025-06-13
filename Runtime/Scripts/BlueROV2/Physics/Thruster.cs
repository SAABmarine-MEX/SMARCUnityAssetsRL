using Unity.Mathematics;
// TODO: check is this even used? check when going over the scenes. if not used, remove

namespace DefaultNamespace
{
    public class ThrusterT200ESC
    {
        private int pwm = 1500;

        double PWMToForce1()
        {
            double force = 0;
            return force;
        }
        
        double PWMToForce(double pwm)
        {
            // This one is nomrilizaed [-1, 1]
            double force = -140.3*math.pow(pwm,9)+389.9*math.pow(pwm,7)-404.1*math.pow(pwm,5)+176.0*math.pow(pwm,3)+8.9*pwm;
            return force;
        }
    }
}