package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by bodeng on 11/17/18.
 */

public class DoubleSampleCrater extends CustomLinearOpMode {
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initizialize();
        telemetry.addLine("Vuforia initialization complete");

        motorLiftDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftDown2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
        motorExtend.setPower(0);

        motorLiftDown1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!opModeIsActive()) {
            motorLiftDown1.setTargetPosition(0);
            motorLiftDown2.setTargetPosition(0);
            motorLiftDown1.setPower(-.3);
            motorLiftDown2.setPower(-.3);
        }

        motorLiftDown1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftDown2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        delatch();

        getBlock();
        telemetry.addData("blockPos: ", blockPos);
        telemetry.update();

        moveToEncoder(-450, .45, 0);

        lowerLift();

        stopMotors();

        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2000);
            moveToEncoderT(-1000, .55, 45, 2000);
            moveToEncoderT(800, .55, 45, 2000);

        } else if (blockPos == 'C') {
            moveToEncoderT(-900, .55, 0, 2000);
            moveToEncoderT(750, .55, 0, 2000);
        } else {
            Pturn(-45, 2000);
            moveToEncoderT(-1000, .55, -45, 2000);
            moveToEncoderT(800, .55, -45, 2000);
        }

        Pturn(-90, 3000);
        moveToEncoderT(-2100, .45, -90, 3500);
        Pturn(-135, 2000);
        moveToEncoderT(-1500, .55, -135, 3000);
        depositMarker();
        //moveToEncoderT(3000, .85, -139, 5000);

        //from here, go for double sample
        //TODO: i dont think this works :]
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(35,2000); //turn towards right particle
            moveToEncoderT(1000, .55, 35, 2000); //sample
            Pturn(-80, 5000); //turn towards wall?!?
            moveToEncoderT(2000, .5, -35, 3000); //go to wall
            Pturn(35, 2000); //straighten up
        } else if (blockPos == 'C') {
            //moveToEncoderT(500, .5, -135, 1000); //maybe go forward a bit
            Pturn(65,2000); //turn towards center particle
            moveToEncoderT(1000, .55, 65, 3000); //sample
            moveToEncoderT(-1000, .55, 65, 3000); //go back
            Pturn(-65, 2000); //straighten up
        } else {
            //moveToEncoderT(500, .5, -135, 1000); //maybe go forward a bit
            Pturn(85,2000); //turn towards center particle
            moveToEncoderT(1500, .55, 85, 3000); //sample
            moveToEncoderT(-1500, .55, 85, 3000); //go back
            Pturn(-85, 2000); //straighten up
        }
        moveToEncoderT(2500, .85, -139, 5000); //yeet into the crater?
    }
}
