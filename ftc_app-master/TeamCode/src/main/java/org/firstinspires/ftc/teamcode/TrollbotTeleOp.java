package org.firstinspires.ftc.teamcode;

public class TrollbotTeleOp extends CustomOpMode{
    public void init() {
        initizialize();
    }
    public void loop() {
        double y1 = gamepad1.left_stick_y;
        double y2 = gamepad1.right_stick_y;

        if (Math.abs(y1) > 0.1){
            setLeftMotors(y1);
        }
        else
        {
            setLeftMotors(0);
        }

        if (Math.abs(y2) > 0.1){
            setRightMotors(y2);
        }
        else
        {
            setRightMotors(0);
        }

    }

}