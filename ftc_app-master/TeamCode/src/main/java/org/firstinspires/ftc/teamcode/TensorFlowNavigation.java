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
 * Created by ftc-summer on 11/12/18.
 */

public class TensorFlowNavigation extends CustomLinearOpMode {
    // x coordinate of yellow block
    int x = 0;

    // y coordinate of yellow block
    int y = 0;

    // width of image
    int width = 800;

    // height of image
    int height = 600;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // uses the x value as the error angle
        Pturn(x,2000);

        // change the 0 to something else later (however high from the top of the screen)
        while (y > 0)
        {
            //driveForward();
        }

    }


}
