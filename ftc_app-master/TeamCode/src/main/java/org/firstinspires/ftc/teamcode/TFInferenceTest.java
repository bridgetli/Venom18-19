package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.tensorflow.contrib.android.TensorFlowInferenceInterface;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.Arrays;

@Autonomous(name = "TFLiteTest", group = "test")
public class TFInferenceTest extends CustomLinearOpMode {

    public void runOpMode() {
        //TODO instantiate input and output names
        String INPUT_NAME = "input_1";
        String OUTPUT_NAME = "output_1";
        float[] results = new float[1000];

        try {
            //load model and image
            telemetry.addData("Loading", "Keras Model");
            telemetry.update();

            TensorFlowInferenceInterface tfii = new TensorFlowInferenceInterface(hardwareMap.appContext.getAssets().open("roboModel.pb"));

            telemetry.addData("Model Successfully Loaded", "Press Play to Start Inference");
            telemetry.update();

            waitForStart();

            //get vuforia image
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            ByteBuffer image = frame.getImage(0).getPixels();

            //inference
            tfii.feed(INPUT_NAME, image, 28, 28, 1);
            tfii.run(new String[]{OUTPUT_NAME});
            tfii.fetch(OUTPUT_NAME, results);

            telemetry.addData("Results", Arrays.toString(results));
            telemetry.update();

            frame.close();
            tfii.close();
        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
        }
    }
}
