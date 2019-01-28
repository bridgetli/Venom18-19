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
    private char blockPos = 'C';

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

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

        getBlock();
        telemetry.addData("blockPos: ", blockPos);
        telemetry.update();

        moveToEncoder(-550, .45, 0);

        lowerLift();

        sleep(100);

        moveToEncoder(560, .2, 0);
        stopAllMotors();
        sleep(250); //500
        //TODO: optimize paths, focus on depot side first; averages 18 sec currently; goal 15 sec?
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2000);
            moveToEncoderT(-600, .45, 45, 2000);
            moveToEncoderT(600, .45, 45, 2000);

        } else if (blockPos == 'C') {
            moveToEncoderT(-600, .45, 0, 2000);
            moveToEncoderT(600, .45, 0, 2000);
        } else {
            Pturn(-45, 2000);
            moveToEncoderT(-600, .45, -45, 2000);
            moveToEncoderT(600, .45, -45, 2000);
        }

        Pturn(-90, 3000);
        moveToEncoderT(-2100, .45, -90, 3500);
        Pturn(-135, 2000);
        moveToEncoderT(-2000, .55, -135, 3000);
        depositMarker();
        moveToEncoderT(5000, .85, -137, 5000);

    }
}