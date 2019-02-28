package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.os.Environment;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * Created by bodeng on 10/19/18.
 */

@Autonomous (name = "SingleSampleDepot", group = "Autonomous")
public class SingleSampleDepot extends CustomLinearOpMode {

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

        Pturn(0, 1000);
        telemetry.addLine("turn done");

        getBlock();

        //telemetry.addLine(tensorflowInfo);
        telemetry.addData("blockPos: ", blockPos);
        telemetry.update();

        moveToEncoder(-400, .45, 0);

        lowerLift();

        stopMotors();

        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2000);
            //succDaBlock(45);
            moveToEncoderT(-2500, .45, 45, 3000);
            Pturn(-45, 2000);
            moveToEncoderT(-2000, .45, -45, 3000);
            Pturn(45, 2000);
            depositMarker();
            moveToEncoderT(3200, .85, 45, 5000);
        } else if (blockPos == 'C') {
            //succDaBlock(0);
            moveToEncoderT(-4400, .45, 0, 3000);
            Pturn(45, 2000);
            depositMarker();
            moveToEncoderT(3200, .85, 45, 5000);
        } else {
            Pturn(-45, 2000);
            //succDaBlock(-45);
            moveToEncoderT(-2500, .45, -45, 2000);
            Pturn(45, 2000);
            moveToEncoderT(-1500, .45, 45, 3000);
            depositMarker();
            moveToEncoderT(3200, .85, 45, 5000);
        }

        //moveToEncoderT(1500, .95, -90, 3000);
        //sleep(250); //500
        //PturnRight(55, 3000);
        //moveToEncoderT(-400, .95, 55, 2000);
        //moveToEncoderT(-1500, .95,48, 3500);

        //depositMarker();

        //moveTimeP(1450, .95, 43);
    }

    public void succDaBlock(double currAngle) throws InterruptedException {
        //maybe after pushing the block, move back and then grab it with the manip
        moveToEncoderT(400, .5, currAngle, 1500); //TODO: need to figure out how far back we need to be

        //lower manip
        eTime.reset();
        while (eTime.milliseconds() < 1000)
            motorManip.setPower(-.5);
        motorManip.setPower(0);

        //succ
        eTime.reset();
        while (eTime.milliseconds() < 1000) {
            servoLeftManip.setPower(-1);
            servoRightManip.setPower(1);
        }
        servoLeftManip.setPower(0);
        servoRightManip.setPower(0);

        //pull manip back
        eTime.reset();
        while (eTime.milliseconds() < 1000)
            motorManip.setPower(.85);
        motorManip.setPower(0);

        //if you want, maybe figure out how to score sampled mineral; dont personally recommend though as it'd be pretty time extensive...
        //insert control award worthy code here

        //return to original pos, and continue with auto
        moveToEncoderT(-400, .5, currAngle, 1500); //TODO: ditto above... except reverse
    }
}