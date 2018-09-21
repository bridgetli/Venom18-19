package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bodeng on 9/19/18.
 */

public class PIDtesting extends CustomLinearOpMode {


    public void rightTurn(double angle) {
        double kP = 0.2;
        double kI = 0.001;
        double kD = 0.005;
        double angleError = 0;
        double oldTime = 0;
        double totalError = 0;
        double oldError = 0;
        double newTime = 0;
        double I = 0;
        double P = 0;
        double D = 0;
        ElapsedTime timeStuff = new ElapsedTime();
        while (angleError > 0) {
            angleError = angle - imu.getYaw();
            newTime = timeStuff.milliseconds();
            totalError += (newTime - oldTime) * angleError;
            P = kP * angleError;
            I = totalError * kI * angleError;
            D = (angleError - oldError) / (newTime - oldTime) * kD;

            oldTime = newTime;
            oldError = angleError;
            motorFL.setPower(Range.clip(P + I + D, -1, 1));
            motorBL.setPower(Range.clip(P + I + D, -1, 1));
            motorFR.setPower(Range.clip(-P - I - D, -1, 1));
            motorBR.setPower(Range.clip(-P - I - D, -1, 1));
        }


    }

    @Override
    public void runOpMode() {
        initizialize();
        waitForStart();
        rightTurn(30);
    }
}
