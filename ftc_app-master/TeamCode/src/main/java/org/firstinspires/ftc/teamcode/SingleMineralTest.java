package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "Mineral Location Test", group = "tftest")
public class SingleMineralTest extends CustomLinearOpMode {
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
            float x;
            float area;
            //TODO: These are place holders, please remember to adjust
            float rightBounds = 600;
            float leftBounds = 400;
            float areaUpperBounds = 8000;
            float areaLowerBounds = 4000;

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
                            x = (recognition.getRight() + recognition.getLeft() / 2);
                            telemetry.addData("x", x);
                            if (x < rightBounds && x > leftBounds) {
                                telemetry.addLine("No action needed");
                            } else if (x > rightBounds) {
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
}
