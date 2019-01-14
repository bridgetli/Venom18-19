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

        motorLiftDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftDown2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
        motorLiftUp.setPower(0);

        waitForStart();

        delatch();

        sleep(1000);

        stopMotors();

        //TODO: optimize paths, focus on depot side first; averages 18 sec currently; goal 15 sec?
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(55, 2500);
            sleep(200); //500
            moveToEncoder(1000, .65, 55);
            sleep(200);
            moveToEncoder(1000, .65, -30);

        } else if (blockPos == 'C') {
            //Pturn(45, 2500);
            //sleep(500);
            moveToEncoder(1000, .65, 10);


        } else {
            Pturn(-39, 3000);
            sleep(200); //500
            moveToEncoder(1000, .65, -39);
            moveToEncoder(1500, .65, 20);

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