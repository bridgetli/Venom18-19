package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class shreyastest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFL;
        DcMotor motorFR;

        motorFL = hardwareMap.dcMotor.get("motorFL");


        motorFL.setPower(1);
    }
}
