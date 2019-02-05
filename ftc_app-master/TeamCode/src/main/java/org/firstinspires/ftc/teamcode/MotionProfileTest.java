package org.firstinspires.ftc.teamcode;

public class MotionProfileTest extends CustomLinearOpMode {

    //tbh im not even sure if this pseudocode is correct

    /**
     * Idk this is just a test. :)
     * @param pos = position
     * @param maxV = maximum velocity
     * @param maxA = maximum acceleration
     */
    private void sCurve(double pos, double maxV, double maxA) {
        int phase = 1;
        while (averageCurrentVelocity()*time.seconds() < pos) {
            if (phase == 1) {
                while (averageCurrentAcceleration() < maxA)
                    //increase acceleration
                //stop increasing acceleration
                phase = 2;
            }
            if (phase == 2) {
                while (averageCurrentVelocity() < .75*maxV)
                    //keep accelerating at max rate
                phase = 3;
            }
            if (phase == 3) {
                while (averageCurrentAcceleration() > 0 && averageCurrentVelocity() < maxV)
                    //start decelerating
                //should be at max velocity
                phase = 4;
            }
            if (phase == 4) {
                //constant velocity
                while (averageCurrentVelocity()*time.seconds() < pos/2)
                phase = 5;
            }
            if (phase == 5) {
                //start deceleration
                while (averageCurrentAcceleration() > -maxA)
                    //decelerate
                phase = 6;
            }
            if (phase == 6) {
                //stay at max deceleration till velocity reaches a certain point?
                while (averageCurrentVelocity() > .25*maxV)
                    //keep decelerating
                phase = 7;
            }
            if (phase == 7) {
                while (averageCurrentVelocity() > 0)
                    //DECELERATE MAXIMOSO
                //loop should break here
                phase = 0;
            }
        }
        //yay?
    }
}
