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

@Autonomous (name = "SingleSampleCrater", group = "Autonomous")
public class SingleSampleCrater extends CustomLinearOpMode {

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

        Pturn(0, 1000);
        telemetry.addLine("turn done");

        getBlock();
        telemetry.addData("blockPos: ", blockPos);
        telemetry.update();

        moveToEncoder(-400, .45, 0);

        lowerLift();

        stopMotors();

        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2000);
            moveToEncoderT(-900, .55, 45, 2000);
            moveToEncoderT(670, .55, 45, 2000);

        } else if (blockPos == 'C') {
            moveToEncoderT(-900, .55, 0, 2000);
            moveToEncoderT(700, .55, 0, 2000);
        } else {
            Pturn(-45, 2000);
            moveToEncoderT(-900, .55, -45, 2000);
            moveToEncoderT(700, .55, -45, 2000);
        }

        Pturn(-90, 3000);
        moveToEncoderT(-2100, .45, -90, 3500);
        Pturn(-135, 2000);
        moveToEncoderT(-1500, .55, -135, 3000);
        depositMarker();
        moveToEncoderT(3000, .85, -139, 5000);

    }


}