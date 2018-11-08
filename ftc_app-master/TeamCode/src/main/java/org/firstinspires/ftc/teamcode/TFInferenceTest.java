package org.firstinspires.ftc.teamcode;

import org.tensorflow.contrib.android.TensorFlowInferenceInterface;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;

public class TFInferenceTest extends CustomLinearOpMode {

    public static void main(String[] args) {

        try {
            FileInputStream fos = new FileInputStream(".pb file path goes here");

            TensorFlowInferenceInterface tfii = new TensorFlowInferenceInterface(fos);

            tfii.feed("input goes here");
            tfii.run("? something goes here");
            tfii.fetch("parameter from previous method?");

            System.out.println("output results here");
            fos.close();
        } catch(Exception e) {
            System.out.println("ERROR: " + e.toString());
        }
    }

    public void runOpMode() {
        try {
            telemetry.addData("Loading Keras Model", "");
            telemetry.update();

            TensorFlowInferenceInterface tfii = new TensorFlowInferenceInterface(hardwareMap.appContext.getAssets(), "file name goes here");

            telemetry.addData("Model Successfully Loaded", "Starting ");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("ERROR - Program Stopping", e.toString());
        }
        telemetry.update();
    }
}
