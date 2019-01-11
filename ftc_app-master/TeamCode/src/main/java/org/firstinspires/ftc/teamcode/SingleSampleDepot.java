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

        moveToEncoder(560, .45, 0);
        sleep(500);

        //TODO: optimize all of these paths; averages 17 sec currently; maybe get to 15 or less
        if (blockPos == 'R' || blockPos == '?') { //also, default path should be center, as it is slightly faster //ehh don't worry about it hamza
            //turn towards block & move through it
            Pturn(45, 2500);
            sleep(250); //500
            moveToEncoder(1500, .40, 45); //increase encoder/power (if we change path)?
            sleep(250); //500


            Pturn(135, 2500); //replace this; instead, move straight through the block?
            sleep(250); //500
            moveToEncoderT(-1500, .35, 135, 2000);
            Pturn(-135, 2500); //turn to depot (adjust angle or remove statement)

            moveToEncoderT(700, .35, -135, 2000); //adjust angle as needed
            depositMarker();
            moveTimeP(2000, .8, -134);

        } else if (blockPos == 'C') { //why do we not just drive straight through?? //great question
            moveToEncoder(1500, .35, 0); //all the way to depot
            sleep(250); //500

            Pturn(-45, 2000); //turn towards crater
            moveToEncoderT(400, .35, -45, 2000); //move a little forward while turning to align with wall
            Pturn(-135, 2500); //drop marker


            moveToEncoderT(300, .35, -135, 2000);
            depositMarker();
            moveTimeP(2000, .8, -134); //drive to crater

        } else {
            Pturn(-45, 2500);
            sleep(250); //500
            moveToEncoder(1300, .35, -45);
            sleep(250); //500


            Pturn(-135, 2500); //remove, just drive straight to wall? //you can try i guess
            moveTimeP(400, -.35, -135); //turn towards crater, and rest is the same
            //moveToEncoderT(1000, .35, -135, 2000);
            depositMarker();
            moveTimeP(2000, .8, -134);
        }
    }
}