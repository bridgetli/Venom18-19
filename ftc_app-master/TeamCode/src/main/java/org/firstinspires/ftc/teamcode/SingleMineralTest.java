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
        //TODO: uncomment next line and delete 2 init lines
        //initizialize();
        initVuforia();
        initTfod();

        telemetry.addData("Object Location Test Ready", "Press Play to Start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            //activate tensorflow
            if (tfod != null)
                tfod.activate();
            float top;
            float area;
            //TODO: These are place holders, please remember to adjust
            float topBounds = 550;
            float bottomBounds = 350;
            float areaUpperBounds = 900;
            float areaLowerBounds = 250;

            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                        //this if statement is important, probably will need to be changed
                        if (updatedRecognitions.size() == 1) {
                            Recognition recognition = updatedRecognitions.get(0);
                            telemetry.addData("Mineral Type", recognition.getLabel());

                            //center robot with mineral
                            top = recognition.getTop();
                            telemetry.addData("Top", top);
                            if (top < topBounds && top > bottomBounds) {
                                telemetry.addLine("No action needed");
                            } else if (top > topBounds) {
                                //turn left
                                telemetry.addLine("Turn left");
                            } else {
                                //turn right
                                telemetry.addLine("Turn right");
                            }
                            //close in on mineral
                            area = recognition.getHeight() * recognition.getWidth();
                            telemetry.addData("Area", area);
                            if (area < areaUpperBounds && area > areaLowerBounds) {
                                telemetry.addLine("No action needed");
                            } else if (area > areaUpperBounds) {
                                //move back
                                telemetry.addLine("Move back");
                            } else {
                                //move forward
                                telemetry.addLine("Move closer");
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
        //deactivate tensorflow
        if (tfod != null)
            tfod.shutdown();
    }

    //Initialize the Vuforia localization engine.
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //Initialize the Tensor Flow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
