package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.List;

@Autonomous(name = "Mineral Location Test", group = "test")
public class SingleMineralTest extends CustomLinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) {
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    System.out.println("I see a gold mineral");
                                    telemetry.addLine("I see a gold mineral");
                                }
                                else {
                                    System.out.println("I see a white mineral");
                                    telemetry.addLine("I see a white mineral");
                                }
                                System.out.println("Left:" + recognition.getLeft());
                                System.out.println("Right:" + recognition.getRight());
                                System.out.println("Bottom:" + recognition.getBottom());
                                System.out.println("Top:" + recognition.getTop());
                                System.out.println("Degrees to Object:" + recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                System.out.println("\n\n");
                                telemetry.addData("Left", recognition.getLeft());
                                telemetry.addData("Right", recognition.getRight());
                                telemetry.addData("Bottom", recognition.getBottom());
                                telemetry.addData("Top", recognition.getTop());
                                telemetry.addData("Degrees to Object",  recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            }
                            //takePic();
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private Bitmap takePic() throws InterruptedException {
        File dir = Environment.getExternalStorageDirectory();
        File[] files = dir.listFiles();
        int highestFileNum = -1;
        for (File f : files) {
            if (f.getName().contains("Mineral_Location_pic_")) {
                highestFileNum = Math.max(highestFileNum, Integer.parseInt(f.getName().replaceAll("\\D", "")));
            }
        }
        int currFileNum = highestFileNum + 1;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        telemetry.addData("Num images in frame", "" + frame.getNumImages());
        telemetry.update();

        Image image = frame.getImage(0);

        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
        Bitmap bmp = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bmp.copyPixelsFromBuffer(image.getPixels());
        try {
            File sdCard = Environment.getExternalStorageDirectory();
            telemetry.addLine(sdCard.getAbsolutePath());
            telemetry.update();

            File file = new File(sdCard, "Mineral_Location_pic_" + currFileNum++ + ".png");

            FileOutputStream fos = new FileOutputStream(file);

            bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);

        } catch (FileNotFoundException ex) {
            telemetry.addLine(ex.toString());
            telemetry.update();
        }

        return bmp;
    }
}
