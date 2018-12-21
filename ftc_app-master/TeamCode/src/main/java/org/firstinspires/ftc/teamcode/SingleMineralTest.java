package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name = "Mineral Location Test", group = "tftest")
public class SingleMineralTest extends CustomLinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        //initizialize(); once you can test this on the robot

        initVuforia();
        initTfod();

        telemetry.addData("Object Location Test Ready", "Press Play to Start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null)
                tfod.activate();

            float top;
            float left;

            //TODO: adjust these as needed; make sure to retest on phone mount when possible
            int topBounds = 550;
            int bottomBounds = 350;
            int leftBounds = 250;
            int rightBounds = 150;

            while (opModeIsActive() /*|| !centered*/) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 1) {
                            Recognition recognition = updatedRecognitions.get(0);
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                telemetry.addLine("I see a gold mineral");
                            else
                                telemetry.addLine("I see a white mineral");
                            telemetry.addData("Top", recognition.getTop());
                            telemetry.addData("Left", recognition.getLeft());

                            //TODO: move to mineral
                            top = recognition.getTop();
                            if (top < topBounds && top > bottomBounds) {
                                telemetry.addLine("No action needed");
                            } else if (top > topBounds) {
                                //turn left
                                //turn(2.5);
                                telemetry.addLine("Turn left");
                            } else {
                                //turn right
                                //turn(-2.5);
                                telemetry.addLine("Turn Right");
                            }
                            //wait(100);

                            left = recognition.getLeft();
                            if (left < leftBounds && left > rightBounds) {
                                telemetry.addLine("No action needed");
                            } else if (left > leftBounds) {
                                //back up
                                //moveToDistance(-2.5);
                                telemetry.addLine("Move forwards?");
                            } else {
                                //move forward
                                //moveToDistance(2.5);
                                telemetry.addLine("Move backwards?");
                            }
                            //wait(100);
                            //*/
                        }
                    }
                }
                telemetry.update();
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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
