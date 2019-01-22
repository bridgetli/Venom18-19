package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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
            float top;
            float area;
            //TODO: These are place holders, please remember to adjust
            float topBounds = 550;
            float bottomBounds = 350;
            float areaUpperBounds = 90000;
            float areaLowerBounds = 60000;

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
}
