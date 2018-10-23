package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        telemetry.addData("Right motor speeds", yR);
        telemetry.addData("Left motor speed", yL);
        telemetry.addData("Distance:", getDist());
        telemetry.addLine("Init complete");
        telemetry.update();
    }
}
