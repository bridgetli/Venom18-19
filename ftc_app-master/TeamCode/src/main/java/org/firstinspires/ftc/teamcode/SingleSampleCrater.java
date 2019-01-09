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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        initizialize();
        telemetry.addLine("Vuforia initialization complete");

        waitForStart();

        getBlock();

        telemetry.addData("Block Pos", blockPos);
        telemetry.update();

        sleep(100);

        moveToEncoder(560, .2, 0);
        stopAllMotors();
        sleep(250); //500
        //TODO: optimize paths, focus on depot side first; averages 18 sec currently; goal 15 sec?
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2500);
            sleep(200); //500
            moveToEncoder(500, .35, 45);
            sleep(200);
            moveToEncoder(-480, .35, 45);
            Pturn(-90, 2500);
        } else if (blockPos == 'C') {
            //Pturn(45, 2500);
            //sleep(500);
            moveToEncoder(330, .35, 0);
            sleep(200);
            moveToEncoder(-320, .35, 0);
            Pturn(-90, 2500);
        } else {
            Pturn(-45, 2500);
            sleep(200); //500
            moveToEncoder(500, .35, -45);
            sleep(200);
            moveToEncoder(-480, .35, -45);
            Pturn(-90, 2500);
        }

        moveToEncoderT(2100, .3, -90, 4000);
        sleep(250); //500
        Pturn(45, 2500);
        moveToEncoderT(-1200, .3,46, 4000);

        depositMarker();

        moveTimeP(1450, .6, 43);
    }
}