package org.firstinspires.ftc.teamcode.tflite;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CustomLinearOpMode;
import org.tensorflow.lite.Interpreter;
import java.io.FileInputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

@Autonomous(name = "TFLiteSingleImageInference", group = "test")
public class TFLiteSingleImageInference extends CustomLinearOpMode {

    @Override
    public void runOpMode() {
        float[][] probabilities = new float[1][10];
        Bitmap bmp;
        ByteBuffer buffer;
        Interpreter tflite;
        StringBuilder results;

        try {
            bmp = BitmapFactory.decodeStream(hardwareMap.appContext.getAssets().open("fashion-mnist-sprite.png"));
            buffer = convertBitmapToByteBuffer(bmp);
            tflite = new Interpreter(loadModelFile(hardwareMap.appContext)); //TODO test if this works as read-only buffer
        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
            return;
        }

        telemetry.addData("Press Play to Start", "Tensorflow Inference");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            results = new StringBuilder();
            try {
                tflite.run(buffer, probabilities);
                for (int i = 0; i < 10; i++) {
                    results.append("Label ").append(i).append(": ").append(probabilities[0][i]).append("\n");
                }

                telemetry.addData("RESULTS", results.toString());
            } catch (Exception e) {
                telemetry.addData("ERROR", e.toString());
            }
            telemetry.update();
        }
        tflite.close();
    }

    private MappedByteBuffer loadModelFile(Context activity) throws Exception {
        MappedByteBuffer model;
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd("roboModel.tflite");
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        model = fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);

        fileDescriptor.close();
        inputStream.close();
        fileChannel.close();

        return model;
    }

    private ByteBuffer convertBitmapToByteBuffer(Bitmap bitmap) {
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(28 * 28 * 4);
        byteBuffer.order(ByteOrder.nativeOrder());
        int[] intValues = new int[28 * 28];
        bitmap.getPixels(intValues, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        int pixel = 0;
        for (int i = 0; i < 28; ++i) {
            for (int j = 0; j < 28; ++j) {
                final int val = intValues[pixel++];
                byteBuffer.putFloat(((val >> 16) & 0xFF));
                byteBuffer.putFloat(((val >> 8) & 0xFF));
                byteBuffer.putFloat((val & 0xFF));
            }
        }
        return byteBuffer;
    }
}
