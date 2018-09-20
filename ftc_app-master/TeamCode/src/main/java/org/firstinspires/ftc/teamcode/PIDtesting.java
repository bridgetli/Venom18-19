package org.firstinspires.ftc.teamcode;

/**
 * Created by bodeng on 9/19/18.
 */

public class PIDtesting extends CustomLinearOpMode {

    public void rightTurn(double angle) {
        double kP = 0.5;
        double currentAngle = angle - imu.getYaw();
        double P = kP * currentAngle;


    }

    @Override
    public void runOpMode() {
        initizialize();
        waitForStart();
        motorFL.setPower(1);
        motorBL.setPower(1);
        motorFR.setPower(-1);
        motorBR.setPower(-1);
        rightTurn(30);
    }
}
