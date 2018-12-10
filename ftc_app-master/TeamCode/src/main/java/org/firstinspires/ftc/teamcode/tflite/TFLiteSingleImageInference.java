package org.firstinspires.ftc.teamcode.tflite;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.CustomLinearOpMode;
import org.tensorflow.lite.Interpreter;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Arrays;

@Autonomous(name = "TFLiteSingleImageInference", group = "test")
public class TFLiteSingleImageInference extends CustomLinearOpMode {

    private MappedByteBuffer tfliteModel;
    private Interpreter tflite;
    private ByteBuffer imgData = null;
    private int[] intValues;
    private float[][] labelProbArray = null;
    private VuforiaLocalizer vuforia;
    private String MODEL_NAME = "roboModel.tflite";
    private String IMAGE_FILE = "fashion-mnist-sprite-GRAY.png";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        try {
            tfliteModel = loadModelFile(hardwareMap.appContext.getApplicationContext());
            tflite = new Interpreter(tfliteModel);

            Bitmap bmp = BitmapFactory.decodeStream(hardwareMap.appContext.getAssets().open(IMAGE_FILE));
            convertBitmapToByteBuffer(bmp);
            telemetry.addLine("Model & Image Successfully Loaded");
        } catch (Exception e) {
            telemetry.addData("Model not loaded", e.toString());
        }
        telemetry.update();

        while(opModeIsActive()) {
            try {
                tflite.run(imgData, labelProbArray);
                telemetry.addData("Results", Arrays.toString(labelProbArray));
            } catch (Exception e) {
                telemetry.addData("Couldn't run inference", e.toString());
            }
            telemetry.update();
        }
        tflite.close();
    }

    private MappedByteBuffer loadModelFile(Context activity) throws IOException {
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd(MODEL_NAME);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private void convertBitmapToByteBuffer(Bitmap bitmap) {
        if (imgData == null) {
            return;
        }
        imgData.rewind();
        intValues = new int[28*28];
        bitmap.getPixels(intValues, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        // Convert the image to floating point.
        int pixel = 0;
        for (int i = 0; i < 28; ++i) {
            for (int j = 0; j < 28; ++j) {
                final int val = intValues[pixel++];
                imgData.putFloat(((val >> 16) & 0xFF) / 255.f);
                imgData.putFloat(((val >> 8) & 0xFF) / 255.f);
                imgData.putFloat((val & 0xFF) / 255.f);
            }
        }
    }
}