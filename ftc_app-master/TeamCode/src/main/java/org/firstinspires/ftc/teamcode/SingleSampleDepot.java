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
        telemetry.addLine(tensorflowInfo);
        telemetry.addData("blockPos: ", blockPos);
        telemetry.update();

        moveToEncoder(-550, .45, 0);

        lowerLift();

        stopMotors();

        //TODO: optimize paths, focus on depot side first; averages 18 sec currently; goal 15 sec?
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2000);
            moveToEncoderT(-2500, .45, 45, 3000);
            Pturn(-45, 2000);
            moveToEncoderT(-2000, .45, -45, 3000);
            depositMarker();
            Pturn(45, 2000);
            moveToEncoderT(5000, .85, 45, 5000);
        } else if (blockPos == 'C') {
            moveToEncoderT(-4350, .45, 0, 3000);
            depositMarker();
            Pturn(45, 2000);
            moveToEncoderT(5000, .85, 45, 5000);
        } else {
            Pturn(-45, 2000);
            moveToEncoderT(-2500, .45, -45, 2000);
            Pturn(45, 2000);
            moveToEncoderT(-1500, .45, 45, 3000);
            depositMarker();
            moveToEncoderT(5000, .85, 45, 5000);
        }

        //moveToEncoderT(1500, .95, -90, 3000);
        //sleep(250); //500
        //PturnRight(55, 3000);
        //moveToEncoderT(-400, .95, 55, 2000);
        //moveToEncoderT(-1500, .95,48, 3500);

        //depositMarker();

        //moveTimeP(1450, .95, 43);
    }


}