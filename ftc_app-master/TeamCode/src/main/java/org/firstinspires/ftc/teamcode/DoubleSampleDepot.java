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

@Autonomous (name = "DoubleSampleDepot", group = "Autonomous")
public class DoubleSampleDepot extends CustomLinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    private char blockPos = 'C';

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

        telemetry.addData("Block Pos", blockPos);
        telemetry.update();

        sleep(100);

        //go straight a little to get out from under the lander; may need to be adjusted if it hits multiple particles
        moveToEncoder(580, .2, 0);
        sleep(1000);
        
        // If you're trying to tune this part, which will probably take most of your time,
        // Watch for the sleeps; they divide each motion so you can keep track of what you're looking at
        if (blockPos == 'R' || blockPos == '?') {
            Pturn(45, 2500);                     //turn towards block
            sleep(500);
            moveToEncoder(1500, .35, 45);   //move straight to knock block off
            sleep(500);
            Pturn(135, 2500);                    //turn to line up with wall
            sleep(500);
            moveTimeP(400, -.35, 135);      //move to get into position to deposit marker
            depositMarker();
            //moveToEncoder(1350, .25, 135);
        } else if (blockPos == 'C') {
            moveToEncoder(1500, .25, 0); //move straight to knock block off
            sleep(500);
            Pturn(45, 2000);                    //turn to line up with wall
            moveToEncoderT(300, .35, 45, 2000); //move to wall
            sleep(500);
            Pturn(135, 2500);                   //turn to line up with wall again
            moveTimeP(800, -.4, 135);       //ok actually i forgot what this does exactly just watch it i guess
            moveToEncoder(1000, .25, 135);
            depositMarker();
            //moveToEncoder(1350, .25, 135);
        } else {
            Pturn(-45, 2500);                   //turn towards block
            sleep(500);
            moveToEncoder(1300, .35, -45);  //move straight to knock block off
            sleep(500);
            Pturn(45, 2500);                    //wow i'm rusty i guess just watch this too
            moveTimeP(1300, .4, 45);
            Pturn(135, 2500);
            moveToEncoder(600, .25, 135);
            depositMarker();
            //moveToEncoder(1350, .25, 135);
        }

        // Moves robot along the wall until it reaches the line at which it turns for double sampling
        // If the robot doesn't go far enough, increase yIntercept
        // If it goes too far, decrease yIntercept
        // Method itself probably doesn't need to be tuned, but you can try I guess
        moveToLineP(45, 135, 4500);

        // Double sampling sections
        // Should be ok, probably let Bo know before changing these

        if (blockPos == 'L') {
            //Pturn(45, 2000);
            //moveToEncoder(200, .4, 45);
            moveTimeP(1500, .3, 150);
            Pturn(70, 1000);
            moveTimeP(500, .4, 70);
        } else if (blockPos == 'C') {


            Pturn(180, 2000);
            sleep(500);
            moveToEncoder(1700, .35, 180);

            Pturn(90, 2000);
            moveTimeP(1000, .4, 90);
        } else {


            Pturn(180, 2000);
            sleep(500);
            moveToEncoder(2200, .35, 180);

            Pturn(90, 2000);
            moveTimeP(1000, .4, 90);
        }
    }
}