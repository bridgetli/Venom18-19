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

    private ByteBuffer imgData = ByteBuffer.allocateDirect(4 * 28 * 28);
    private float[][] labelProbArray = new float[1][10];

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        MappedByteBuffer tfliteModel;
        Interpreter tflite = null;
        String IMAGE_PATH = "fashion-mnist-sprite-GRAY.png";
        Bitmap bmp;

        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        boolean modelLoaded = false, imgLoaded = false;
        while(!modelLoaded || !imgLoaded) {
            if (!modelLoaded) {
                try {
                    tfliteModel = loadModelFile(hardwareMap.appContext.getApplicationContext());
                    tflite = new Interpreter(tfliteModel);

                    telemetry.addLine("Model Successfully Loaded");
                    modelLoaded = true;
                } catch (Exception e) {
                    telemetry.addData("Model not loaded", e.toString());
                }
            }
            if (!imgLoaded) {
                try {
                    bmp = BitmapFactory.decodeStream(hardwareMap.appContext.getAssets().open(IMAGE_PATH));
                    convertBitmapToByteBuffer(bmp);

                    telemetry.addLine("Image Successfully Loaded");
                    imgLoaded = true;
                } catch (Exception e) {
                    telemetry.addData("Image not loaded", e.toString());
                }
            }
            telemetry.update();
        }

        while(opModeIsActive()) {
            try {
                tflite.run(imgData, labelProbArray);
                for (float[] resultArray : labelProbArray) {
                    for (float result : resultArray)
                        telemetry.addData("Probability", result);
                }
            } catch (Exception e) {
                telemetry.addData("Couldn't run inference", e.toString());
            }
            telemetry.update();
        }
        tflite.close();
    }

    private MappedByteBuffer loadModelFile(Context activity) throws IOException {
        String MODEL_PATH = "roboModel.tflite";
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd(MODEL_PATH);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private void convertBitmapToByteBuffer(Bitmap bitmap) {
        int[] intValues = new int[28*28];
        imgData.rewind();
        bitmap.getPixels(intValues, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        // Convert the image to floating point.
        int pixel = 0;
        for (int i = 0; i < 28; ++i) {
            for (int j = 0; j < 28; ++j) {
                final int val = intValues[pixel++];
                addPixelValue(val);
            }
        }
    }

    private void addPixelValue(int pixelValue) {
        imgData.put((byte) ((pixelValue >> 16) & 0xFF));
        imgData.put((byte) ((pixelValue >> 8) & 0xFF));
        imgData.put((byte) (pixelValue & 0xFF));
        /*
        imgData.putFloat(((pixelValue >> 16) & 0xFF) / 255.f);
        imgData.putFloat(((pixelValue >> 8) & 0xFF) / 255.f);
        imgData.putFloat((pixelValue & 0xFF) / 255.f); */
    }
}