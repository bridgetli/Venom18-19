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

        waitForStart();

        getBlock();

        motorLiftDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftDown2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
        motorExtend.setPower(0);

        motorLiftDown1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveToEncoder(560, .2, 0);
        sleep(500);

        //TODO: optimize all of these paths; averages 17 sec currently; maybe get to 15 or less
        if (blockPos == 'R' || blockPos == '?') { //also, default path should be center, as it is slightly faster //ehh don't worry about it hamza
            //turn towards block & move through it
            Pturn(45, 2500);
            sleep(250); //500
            moveToEncoder(1500, .35, 45); //increase encoder/power (if we change path)?
            sleep(250); //500
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

        } else if (blockPos == 'C') { //why do we not just drive straight through?? //great question
            moveToEncoder(1500, .25, 0); //all the way to depot
            sleep(250); //500


            Pturn(-45, 2000); //turn towards crater
            moveToEncoderT(400, .35, -45, 2000); //move a little forward while turning to align with wall
            Pturn(-135, 2500); //drop marker
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
}