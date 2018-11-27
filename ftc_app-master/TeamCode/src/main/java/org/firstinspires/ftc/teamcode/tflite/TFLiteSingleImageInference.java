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
import java.util.Arrays;

@Autonomous(name = "TFLiteSingleImageInference", group = "test")
public class TFLiteSingleImageInference extends CustomLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        float[][] result = new float[1][10];
        Bitmap bmp;
        ByteBuffer buffer;
        Interpreter tflite;

        telemetry.addData("Press Play to Start", "Tensorflow Inference");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            try {
                bmp = BitmapFactory.decodeStream(hardwareMap.appContext.getAssets().open("fashion-mnist-sprite.png"));
                buffer = convertBitmapToByteBuffer(bmp);
                tflite = new Interpreter(loadModelFile(hardwareMap.appContext));

                tflite.run(buffer, result);
                telemetry.addData("Results", Arrays.toString(result));
            } catch (Exception e) {
                telemetry.addData("ERROR", e.toString());
            }
            telemetry.update();
        }
    }

    private MappedByteBuffer loadModelFile(Context activity) throws Exception {
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd("roboModel.tflite");
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
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
                byteBuffer.put((byte) ((val >> 16) & 0xFF));
                byteBuffer.put((byte) ((val >> 8) & 0xFF));
                byteBuffer.put((byte) (val & 0xFF));
            }
        }
        return byteBuffer;
    }
}
