/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "DriverlessTest", group = "Concept")
public class DriverlessTest extends CustomOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AU1EPdr/////AAABmT6zWfr8qUNugR5o7PvwzcEJfcXKCLInER6PgCU4kiAwOmPTqEJB9HCG9hlVk009cFlQbSYCfySClawEGv8sVVlYagXM4pXlFGtqw+gDH7+Y35RYUp5aZzm++TPT/Zgd3uJSd2FNtQKXqCFqWp0kar/a50Q5B3kE3cWw6+UFaYTNSSSgDVtMNkZgu4fCbgpIo8iOCQnaOJUsxdo41Nt/VdkaQ2+78ys2EJOkSEAw8lvWSRU4XXBc3p3e8NrSXIjpxUGUIYAIZ7rsvxH2ck3qEcBu+KyRWGzSk5xGAfXY8+2AQHaSMpYanZt2k2d68ROZuwog30HcWwpSfueDw3NuWbN+WIi5XicgbiTunHUlXQiD";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    ElapsedTime eTime = new ElapsedTime();

    double kP = .6/90;
    double minSpeed = .38;
    double maxSpeed = 1;
    boolean locked = false;
    String mode = "";
    double desiredAngle = 0;

    @Override
    public void init() {
        //TODO: eventually move initVuforia() and initTfod() into CustomOpMode as part of initizialize()
        initizialize();
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addLine("If you see this error message, you are definitely screwing something up.");
        }

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        if (tfod != null) {
            tfod.activate();
        }
    }

    @Override
    public void loop() {
        int golds = 0, silvers = 0;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition recognition, Recognition t1) {
                        if (recognition.getBottom() > t1.getBottom())
                            return -1;
                        return 1;
                    }
                });


                telemetry.addData("# Object Detected", updatedRecognitions.size());
                //Marcus is the best hardware lead -Bo Deng
                //He is also god tier at CAD, I want him next year on my team
                for (Recognition particle : updatedRecognitions) {
                    telemetry.addLine("NEW PARTICLE\n");
                    telemetry.addData("Type: ", particle.getLabel());
                    telemetry.addData("Angle to particle: ", particle.estimateAngleToObject(AngleUnit.DEGREES));
                    telemetry.addData("x of particle: ", (particle.getLeft() + particle.getRight()) / 2);
                    telemetry.addData("y of particle: ", (particle.getTop() + particle.getBottom()) / 2);
                    telemetry.addData("Confidence: ", particle.getConfidence());
                    telemetry.addLine("\n\n");

                    if (particle.getLabel().equals(LABEL_GOLD_MINERAL)) golds++;
                    else silvers++;
                }
                telemetry.update();

                if (gamepad1.a) {
                    if (!locked  && !updatedRecognitions.isEmpty()) {
                        desiredAngle = imu.getYaw() + updatedRecognitions.get(0).estimateAngleToObject(AngleUnit.DEGREES) - 14;
                        locked = true;
                        mode = "turning";
                        if (desiredAngle > 180)
                            desiredAngle -= 360;
                        else if (desiredAngle <= -180)
                            desiredAngle += 360;

                        eTime.reset();

                    } else if (mode.equals("turning") && eTime.milliseconds() < 3500) {

                        double angleError = imu.getTrueDiff(desiredAngle);

                        double PIDchange = kP * angleError;

                        if (PIDchange > 0 && PIDchange < minSpeed)
                            PIDchange = minSpeed;
                        else if (PIDchange < 0 && PIDchange > -minSpeed)
                            PIDchange = -minSpeed;

                        motorBL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
                        motorFL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
                        motorBR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));
                        motorFR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));

                        telemetry.addData("angleError: ", angleError);
                        telemetry.addData("PIDCHANGE: ", PIDchange);
                        telemetry.update();

                    } else if (mode.equals("turning")) {

                        stopMotors();
                        eTime.reset();
                        mode = "closing";

                    } else if (mode.equals("closing") && !updatedRecognitions.isEmpty()) {
                        double currY = updatedRecognitions.get(0).getBottom();
                        if (currY < 400 && eTime.milliseconds() < 2000) {
                            motorBL.setPower(-.5);
                            motorBR.setPower(-.5);
                            motorFL.setPower(-.5);
                            motorFR.setPower(-.5);
                        } else {
                            stopMotors();
                            mode = "picking up";
                            eTime.reset();
                        }
                    } else if (mode.equals("picking up")) {
                        if (eTime.milliseconds() < 1000) {
                            motorManip.setPower(-.5);
                        } else {

                            motorManip.setPower(0);

                            mode = "extending";
                            eTime.reset();
                        }
                    } else if (mode.equals("extending")) {

                        servoLeftManip.setPower(.8);
                        servoRightManip.setPower(-.8);

                        if (eTime.milliseconds() < 2500) {
                            motorExtend.setPower(1);

                        } else {
                            motorExtend.setPower(0);
                            mode = "retracting";
                            eTime.reset();
                        }
                    } else if (mode.equals("retracting")) {

                        if (eTime.milliseconds() < 2500) {
                            motorExtend.setPower(-1);
                        } else {
                            motorExtend.setPower(0);
                            mode = "resetting";
                            eTime.reset();
                        }
                    } else if (mode.equals("resetting")) {
                        servoLeftManip.setPower(0);
                        servoRightManip.setPower(0);

                        if (eTime.milliseconds() < 2000) {
                            motorManip.setPower(.85);
                        } else {
                            motorManip.setPower(0);
                            mode = "";
                            locked = false;
                            telemetry.addLine("done, let go");
                        }
                    }
                    telemetry.addData("Mode: ", mode);
                }
                //(Y) for gold cube, (X) for white ball
                /*if (golds > 0 && gamepad1.y) {
                    trackMineral(closestMineral(LABEL_GOLD_MINERAL, updatedRecognitions));
                } else if (silvers > 0 && gamepad1.x) {
                    trackMineral(closestMineral(LABEL_SILVER_MINERAL, updatedRecognitions));
                }*/
            }
        }
    }

    //finds closest mineral of specified type
    private Recognition closestMineral(String label, List recognitions) {
        Recognition temp;
        for (Object recognition : recognitions) {
            temp = (Recognition) recognition;
            if (temp.getLabel().equals(label)) return temp;
        }
        //it shouldn't ever make it to this statement...
        return (Recognition) recognitions.get(0);
    }

    //This method tracks and grabs mineral. Runs until mineral is grabbed or (B) is pressed.
    private void trackMineral(Recognition recognition) {
        float x;
        float y;
        float area;
        boolean Xcheck, Acheck, done = false;

        //TODO: Please adjust these, they aren't that accurate
        float rightBounds = 600;
        float leftBounds = 400;

        float upperBound = 200;
        float lowerBound = 500;

        float areaUpperBounds = 8000;
        float areaLowerBounds = 4000;

        telemetry.addData("Tracking", recognition.getLabel());
        while (!done && !gamepad1.b /*&& opModeActive()*/) {
            //center robot with mineral
            x = (recognition.getRight() + recognition.getLeft()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            if (x < rightBounds && x > leftBounds) {
                telemetry.addLine("No action needed");
                Xcheck = true;
            } else if (x > rightBounds) {
                //turn left
                telemetry.addLine("Turn right");
                Xcheck = false;
            } else {
                //turn right
                telemetry.addLine("Turn left");
                Xcheck = false;
            }
            //close in on mineral
            area = recognition.getHeight() * recognition.getWidth();            //TODO: Change to simple bottom bound recognition
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
            //if centered, grab mineral
            if (Xcheck && Acheck) {
                telemetry.addData("Grabbing", recognition.getLabel());
                //TODO: actually grab mineral w/ manipulator
                done = true;
            }
            telemetry.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void Pturn(double angle, int msTimeout) {

        eTime.reset();

        while (Math.abs(imu.getTrueDiff(angle)) > .5 && eTime.milliseconds() < msTimeout) {
            double angleError = imu.getTrueDiff(angle);

            double PIDchange = kP * angleError;

            if (PIDchange > 0 && PIDchange < minSpeed)
                PIDchange = minSpeed;
            else if (PIDchange < 0 && PIDchange > -minSpeed)
                PIDchange = -minSpeed;

            motorBL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
            motorFL.setPower(Range.clip(-PIDchange, -maxSpeed, maxSpeed));
            motorBR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));
            motorFR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));

            telemetry.addData("angleError: ", angleError);
            telemetry.addData("PIDCHANGE: ", PIDchange);
            telemetry.update();
        }
        stopMotors();
    }
    public void stopMotors() {
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }
}
