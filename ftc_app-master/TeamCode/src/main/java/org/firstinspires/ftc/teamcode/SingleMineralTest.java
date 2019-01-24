package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Mineral Location Test", group = "tftest")
public class SingleMineralTest extends CustomLinearOpMode {
    @Override
    public void runOpMode() {
        //TODO: uncomment next line and delete 2 init methods
        //initizialize();
        initVuforia();
        initTfod();

        telemetry.addData("Object Location Test Ready", "Press Play to Start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null)
                tfod.activate();
            int gold, silver;
            ArrayList<Integer> golds = new ArrayList<>(), silvers = new ArrayList<>();
            while (opModeIsActive()) {
                gold = 0;
                silver = 0;
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        for (int i = 0; i < updatedRecognitions.size(); i++) {
                            if (updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                gold++;
                                golds.add(i);
                            } else {
                                silver++;
                                silvers.add(i);
                            }
                        }
                        telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                        telemetry.addData("# of Gold Cubes Detected", gold);
                        telemetry.addData("# of Silver Balls Detected", silver);
                        telemetry.update();

                        //this if statement is important, probably will need to be changed
                        //(Y) for gold, (X) silver
                        if (gold != 0 && gamepad1.y) {
                            trackMineral(updatedRecognitions.get(golds.get(0)));
                        } else if (silver != 0 && gamepad1.x) {
                            trackMineral(updatedRecognitions.get(silvers.get(0)));
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

    private void trackMineral(Recognition recognition) {
        float x;
        float area;
        boolean Xcheck, Acheck;

        //TODO: These are place holders, please adjust
        float rightBounds = 600;
        float leftBounds = 400;
        float areaUpperBounds = 8000;
        float areaLowerBounds = 4000;

        telemetry.addData("Tracking", recognition.getLabel());

        //center robot with mineral
        x = (recognition.getRight() + recognition.getLeft() / 2);
        telemetry.addData("x", x);
        if (x < rightBounds && x > leftBounds) {
            telemetry.addLine("No action needed");
            Xcheck = true;
        } else if (x > rightBounds) {
            //turn left
            telemetry.addLine("Turn left");
            Xcheck = false;
        } else {
            //turn right
            telemetry.addLine("Turn right");
            Xcheck = false;
        }
        //close in on mineral
        area = recognition.getHeight() * recognition.getWidth();
        telemetry.addData("Area", area);
        if (area < areaUpperBounds && area > areaLowerBounds) {
            telemetry.addLine("No action needed");
            Acheck = true;
        } else if (area > areaUpperBounds) {
            //move back
            telemetry.addLine("Move back");
            Acheck = false;
        } else {
            //move forward
            telemetry.addLine("Move closer");
            Acheck = false;
        }
        if (Xcheck && Acheck) {
            while (opModeIsActive() /*|| cube is not grabbed*/) {
                telemetry.addLine("Grabbing Mineral... Press 'B' to cancel.");
                //Grab mineral
                if (gamepad1.b)
                    break;
            }
        }
    }
}
