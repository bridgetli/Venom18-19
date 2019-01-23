package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "TFStats", group = "tftest")
public class TensorflowCalibration extends CustomLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        telemetry.addLine("Ready to Test Tensorflow, Press Play to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                    //this if statement is important, probably will need to be changed
                    if (updatedRecognitions.size() == 1) {
                        Recognition recognition = updatedRecognitions.get(0);
                        telemetry.addData("Top", recognition.getTop());
                        telemetry.addData("Bottom", recognition.getBottom());
                        telemetry.addData("Right", recognition.getRight());
                        telemetry.addData("Left", recognition.getLeft());
                        telemetry.addData("Height", recognition.getHeight());
                        telemetry.addData("Width", recognition.getWidth());
                        telemetry.addData("Label", recognition.getLabel());
                        telemetry.addData("Confidence", recognition.getConfidence());
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
