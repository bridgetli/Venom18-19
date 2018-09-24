package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bodeng on 9/19/18.
 */

@Autonomous (name = "PIDtesting", group = "PID")
public class PIDtesting extends CustomLinearOpMode {


    public void rightTurn(double angle) {
        double kP = 0.01775;
        double kI = 0;
        double kD = 0;
        //double kD = 0.00005;
        double angleError = imu.getTrueDiff(angle);
        double oldTime = 0;
        double totalError = 0;
        double oldError = 0;
        double newTime = 0;
        double I = 0;
        double P = 0;
        double D = 0;
        ElapsedTime timeStuff = new ElapsedTime();
        while (Math.abs(angleError) > 0 && opModeIsActive()) {
            angleError = imu.getTrueDiff(angle);
            newTime = timeStuff.seconds();
            totalError += (newTime - oldTime) * angleError;
            P = kP * angleError;
            I = totalError * kI * angleError;
            D = -(angleError - oldError) / (newTime - oldTime) * kD;

            oldTime = newTime;
            oldError = angleError;
            motorFL.setPower(Range.clip(P + I + D, -1, 1));
            motorBL.setPower(Range.clip(P + I + D, -1, 1));
            motorFR.setPower(Range.clip(-P - I - D, -1, 1));
            motorBR.setPower(Range.clip(-P - I - D, -1, 1));
            telemetry.addLine("angleError: " + angleError);
        }


    }

    @Override
    public void runOpMode() {
        initizialize();
        double turnAngle = 30;
        telemetry.addData("current angle: ", imu.getYaw());
        telemetry.addData("angleError: ", imu.getTrueDiff(turnAngle));
        telemetry.addLine("Init complete");
        telemetry.update();
        waitForStart();
        rightTurn(turnAngle);
    }
}
