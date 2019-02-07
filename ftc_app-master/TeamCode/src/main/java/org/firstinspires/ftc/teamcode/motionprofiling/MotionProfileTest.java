package org.firstinspires.ftc.teamcode.motionprofiling;

import org.firstinspires.ftc.teamcode.CustomLinearOpMode;

import java.util.ArrayList;

public class MotionProfileTest extends CustomLinearOpMode {

    public static void main(String[] args) {
        /*
        WheelbaseWidth - the width of your robot drive train (again, units don't matter - consistency does)
        WaypointsArrayList - ArrayList containing the needed waypoints
        MaxVel - The maximum velocity the CENTER of the robot will move at - leave yourself some margin as the left and right wheels can independently move faster than the set MaxVel
        MaxAcc - The maximum velocity the CENTER of the robot will accelerate at to achieve the MaxVel.
        Filename - The name of the text file to be written to. (include .txt)
        Time Delta - How often readouts will occur in the written path. 0.1 = readouts every 0.1 seconds.
        */
        double x, y, theta, wheelBaseWidth, maxVel, maxAcc, timedelta;
        String file = "MPTest.txt";

        //TODO: actually test plz
        x = 24;
        y = 56;
        theta = 0;
        wheelBaseWidth = 10;
        maxVel = 20;
        maxAcc = 10;
        timedelta = .1;

        ArrayList<Waypoint> WaypointsArrayList = new ArrayList<>();
        WaypointsArrayList.add(new Waypoint(x, y, theta));

        Trajectory trajectory = new Trajectory(wheelBaseWidth, WaypointsArrayList, maxVel, maxAcc, file, timedelta);
        trajectory.Generate();
        trajectory.toTextFile();
    }

    //tbh im not even sure if this pseudocode is correct
    //update: actually, fuck everything that is not copypasta
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
