package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.tensorflow.contrib.android.TensorFlowInferenceInterface;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.Arrays;

@Autonomous(name = "TFLiteTest", group = "test")
public class TFInferenceTest extends CustomLinearOpMode {

    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Press Play to Start", "Tensorflow Lite Test");
        telemetry.update();
        waitForStart();

        VuforiaLocalizer.CloseableFrame frame;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        while (opModeIsActive()) {
            frame = vuforia.getFrameQueue().poll();
            if (frame != null) {
                Image image = frame.getImage(0);
                runNeuralNet(image.getPixels());
            }
        }
    }

    private void runNeuralNet(ByteBuffer img) {
        ImageUtils imageUtils = new ImageUtils();
        String input_name = "conv2d_1_input";
        String output_name = "activation_1/Softmax";
        float[] image_float, predictions;

        try {
            //load model
            TensorFlowInferenceInterface tfii = new TensorFlowInferenceInterface(hardwareMap.appContext.getAssets().open("roboModel.pb"));

            //load image
            InputStream inputStream = hardwareMap.appContext.getAssets().open("fashion-mnist-sprite.png");
            Bitmap bitmap = BitmapFactory.decodeStream(inputStream);

            //Using the vuforia image doesnt work on the test model because the input size is incorrect
            //Bitmap bitmap = BitmapFactory.decodeByteArray(img.array(), img.arrayOffset(), img.array().length);
            bitmap = imageUtils.processBitmap(bitmap, 28);
            image_float = imageUtils.normalizeBitmap(bitmap, 28, 0.0f, 1.0f); //TODO set this value?
            predictions = new float[(int) tfii.graphOperation(output_name).output(0).shape().size(1)];

            //inference
            tfii.feed(input_name, image_float, 1, 28, 28, 3);
            tfii.run(new String[]{output_name}); //TODO error pops up here: "generic conv implementation doesn't support grouped conv for now"
            tfii.fetch(output_name, predictions);

            telemetry.addData("Results", Arrays.toString(predictions));
            telemetry.update();

            tfii.close();
        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
        }
    }

    private class ImageUtils {
        /**
         * Returns a transformation matrix from one reference frame into another.
         * Handles cropping (if maintaining aspect ratio is desired) and rotation.
         *
         * @param srcWidth Width of source frame.
         * @param srcHeight Height of source frame.
         * @param dstWidth Width of destination frame.
         * @param dstHeight Height of destination frame.
         * @param applyRotation Amount of rotation to apply from one frame to another.
         *  Must be a multiple of 90.
         * @param maintainAspectRatio If true, will ensure that scaling in x and y remains constant,
         * cropping the image if necessary.
         * @return The transformation fulfilling the desired requirements.
         */
        public Matrix getTransformationMatrix(
                final int srcWidth,
                final int srcHeight,
                final int dstWidth,
                final int dstHeight,
                final int applyRotation,
                final boolean maintainAspectRatio) {
            final Matrix matrix = new Matrix();

            if (applyRotation != 0) {
                // Translate so center of image is at origin.
                matrix.postTranslate(-srcWidth / 2.0f, -srcHeight / 2.0f);

                // Rotate around origin.
                matrix.postRotate(applyRotation);
            }

            // Account for the already applied rotation, if any, and then determine how
            // much scaling is needed for each axis.
            final boolean transpose = (Math.abs(applyRotation) + 90) % 180 == 0;

            final int inWidth = transpose ? srcHeight : srcWidth;
            final int inHeight = transpose ? srcWidth : srcHeight;

            // Apply scaling if necessary.
            if (inWidth != dstWidth || inHeight != dstHeight) {
                final float scaleFactorX = dstWidth / (float) inWidth;
                final float scaleFactorY = dstHeight / (float) inHeight;

                if (maintainAspectRatio) {
                    // Scale by minimum factor so that dst is filled completely while
                    // maintaining the aspect ratio. Some image may fall off the edge.
                    final float scaleFactor = Math.max(scaleFactorX, scaleFactorY);
                    matrix.postScale(scaleFactor, scaleFactor);
                } else {
                    // Scale exactly to fill dst from src.
                    matrix.postScale(scaleFactorX, scaleFactorY);
                }
            }

            if (applyRotation != 0) {
                // Translate back from origin centered reference to destination frame.
                matrix.postTranslate(dstWidth / 2.0f, dstHeight / 2.0f);
            }

            return matrix;
        }

        public Bitmap processBitmap(Bitmap source,int size){

            int image_height = source.getHeight();
            int image_width = source.getWidth();

            Bitmap croppedBitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);

            Matrix frameToCropTransformations = getTransformationMatrix(image_width,image_height,size,size,0,false);
            Matrix cropToFrameTransformations = new Matrix();
            frameToCropTransformations.invert(cropToFrameTransformations);

            final Canvas canvas = new Canvas(croppedBitmap);
            canvas.drawBitmap(source, frameToCropTransformations, null);

            return croppedBitmap;
        }

        public float[] normalizeBitmap(Bitmap source,int size,float mean,float std){

            float[] output = new float[size * size * 3];

            int[] intValues = new int[source.getHeight() * source.getWidth()];

            source.getPixels(intValues, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());
            for (int i = 0; i < intValues.length; ++i) {
                final int val = intValues[i];
                output[i * 3] = (((val >> 16) & 0xFF) - mean)/std;
                output[i * 3 + 1] = (((val >> 8) & 0xFF) - mean)/std;
                output[i * 3 + 2] = ((val & 0xFF) - mean)/std;
            }

            return output;
        }
    }
}
