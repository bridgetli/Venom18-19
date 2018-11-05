package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="mainTele", group="TeleOp")
public class mainTele extends CustomOpMode {
    public void init() {
        initizialize();
    }
    public void loop() {

        double yL = -gamepad1.left_stick_y;
        double yR = -gamepad1.right_stick_y;


        double motorScale = 1;

        if (Math.abs(yL) > .2) {
            motorBL.setPower(leftABSMotorVal(yL) * motorScale);
            motorFL.setPower(leftABSMotorVal(yL) * motorScale);
        } else {
            motorBL.setPower(0);
            motorFL.setPower(0);
        }
        if(Math.abs(yR) > .2) {
            motorBR.setPower(rightABSMotorVal(yR) * motorScale);
            motorFR.setPower(rightABSMotorVal(yR) * motorScale);
        } else {
            motorBR.setPower(0);
            motorFR.setPower(0);
        }

        if (gamepad1.dpad_down) {
            motorWR.setPower(-1);
        } else if (gamepad1.dpad_up){
            motorWR.setPower(1);
        } else {
            motorWR.setPower(0);
        }

        if(gamepad1.y) {
            motorWL.setPower(-1);
        } else if (gamepad1.a) {
            motorWL.setPower(1);
        } else {
            motorWL.setPower(0);
        }

        if(gamepad2.a) {
            //servoWinchArm.setPosition(Range.clip(servoWinchArm.getPosition() + .1, 0, 1));
            servoWinchArm.setPosition(servoWinchArmUpPos);
        }
        if(gamepad2.b) {
            //servoWinchArm.setPosition(Range.clip(servoWinchArm.getPosition() - .1, 0, 1));
            servoWinchArm.setPosition(servoWinchArmDownPos);
        }
        telemetry.addData("Right motor speeds", yR);
        telemetry.addData("Left motor speed", yL);
        telemetry.addData("Distance:", getDist());
        telemetry.addLine("Init complete");
        telemetry.addData("Servo Winch Arm Pos", servoWinchArm.getPosition());
        telemetry.update();
    }
}
