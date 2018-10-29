package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bodeng on 10/19/18.
 */

@Autonomous (name = "BoTest", group = "Autonomous")
public class BoTest extends CustomLinearOpMode {    //test for red double depot side

    ElapsedTime time = new ElapsedTime();
    char blockPos = '?';

    @Override
    public void runOpMode() throws InterruptedException {
        initizialize();
        telemetry.addLine("initialization complete");
        waitForStart();

        getBlock();

        moveToDistP(40, 0);
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45);
        } else if (blockPos == 'C') {
            moveToDistP(32, 0);
        } else {
            Pturn(-45);
        }

        // At this point, front of robot should align with corner of lander

    }

    private void Pturn(double angle) {
        double kP = .5/90;
        while (Math.abs(imu.getYaw() - angle) > .5) {
            double angleError = imu.getTrueDiff(angle);
            double PIDchange = kP * angleError;

            motorBL.setPower(PIDchange);
            motorFL.setPower(PIDchange);
            motorBR.setPower(-PIDchange);
            motorFR.setPower(-PIDchange);
        }
    }


    public void moveTime(double msTime, double leftPow, double rightPow) throws InterruptedException {
        time.reset();
        motorBL.setPower(leftPow);
        motorFL.setPower(leftPow);
        motorBR.setPower(rightPow);
        motorFR.setPower(rightPow);
        while (time.milliseconds() < msTime) {}
        stopMotors();
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }

    public void moveToDistP(double inches, double angle) {
        double kPdist = .15;
        double kPangle = .25/90;
        while ((Math.abs(getDist() - inches) > .25 || imu.getTrueDiff(angle) > .5) && opModeIsActive()) {

            double distError = inches - getDist();
            double PIDchangeDist = kPdist * distError;

            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            motorBL.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorBR.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
        }
        stopMotors();
    }

    public void getBlock() {
        blockPos = 'C';
    }
}
