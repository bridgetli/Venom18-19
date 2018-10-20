package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RealTeleOp", group="Mr. Krabs")
public class RealTeleOp extends CustomOpMode{
    public void init() {
        initizialize();
    }
    public void loop() {
        /*double y1 = -gamepad1.left_stick_y;
        double y2 = gamepad1.right_stick_y;

        if (Math.abs(y1) > 0.1){
            setLeftMotors(y1);
        } else {
            setLeftMotors(0);
        }

        if (Math.abs(y2) > 0.1){
            setRightMotors(y2);
        } else {
            setRightMotors(0);
        } */


        double yL = -gamepad1.left_stick_y;
        double yR = -gamepad1.right_stick_y;
        double motorScale = 1;

        if (Math.abs(yL) > .2 || Math.abs(yR) > .2) {

            motorBL.setPower(leftABSMotorVal(yL) * motorScale);
            motorFL.setPower(leftABSMotorVal(yL) * motorScale);

            motorBR.setPower(rightABSMotorVal(yR) * motorScale);
            motorFR.setPower(rightABSMotorVal(yR) * motorScale);

        } else {
            stopDriveMotors();
        }
        telemetry.addData("Right motor speeds", yR);
        telemetry.addData("Left motor speed", yL);
        telemetry.addData("Distance:", getDist());
        telemetry.addLine("Init complete");
        telemetry.update();
    }

}