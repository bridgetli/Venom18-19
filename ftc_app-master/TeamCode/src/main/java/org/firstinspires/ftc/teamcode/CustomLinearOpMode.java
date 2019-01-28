package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

//import for_camera_opmodes.LinearOpModeCamera;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static android.graphics.Color.blue;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class CustomLinearOpMode extends LinearOpMode {

    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    protected static final String VUFORIA_KEY = "AU1EPdr/////AAABmT6zWfr8qUNugR5o7PvwzcEJfcXKCLInER6PgCU4kiAwOmPTqEJB9HCG9hlVk009cFlQbSYCfySClawEGv8sVVlYagXM4pXlFGtqw+gDH7+Y35RYUp5aZzm++TPT/Zgd3uJSd2FNtQKXqCFqWp0kar/a50Q5B3kE3cWw6+UFaYTNSSSgDVtMNkZgu4fCbgpIo8iOCQnaOJUsxdo41Nt/VdkaQ2+78ys2EJOkSEAw8lvWSRU4XXBc3p3e8NrSXIjpxUGUIYAIZ7rsvxH2ck3qEcBu+KyRWGzSk5xGAfXY8+2AQHaSMpYanZt2k2d68ROZuwog30HcWwpSfueDw3NuWbN+WIi5XicgbiTunHUlXQiD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorExtend;
    DcMotor motorLiftDown1;
    DcMotor motorLiftDown2;
    DcMotor motorManip;

    //speed
    double left = 1.00;

    //winch motors???
    //DcMotor motorWinchUp;
    //DcMotor motorWinchDown;

    ModernRoboticsI2cRangeSensor rangeSensorB;
    ModernRoboticsI2cRangeSensor rangeSensorL;

    Servo servoWinchArm;
    final double servoWinchArmInitPos = .5;
    final double servoWinchArmDepositPos = 1;

    final double winchDownPower = .5;
    final double winchUpPower = .5;


    String tensorflowInfo = "";
    //Servo servoMarker;


    IMU imu;
    ElapsedTime time = new ElapsedTime();
    char blockPos = 'C';

    //just had to put these to run the code dw about it


    @Override
    public void runOpMode() throws InterruptedException {

    }


    // initzialization method
    public void initizialize() {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        //motorWinchUp = hardwareMap.dcMotor.get("motorWinchUp");
        //motorWinchDown = hardwareMap.dcMotor.get("motorWinchDown");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorLiftDown1 = hardwareMap.dcMotor.get("motorLiftDown1");
        motorLiftDown2 = hardwareMap.dcMotor.get("motorLiftDown2");
        //motorWinchUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorWinchDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWinchUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWinchDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoWinchArm = hardwareMap.servo.get("servoWinchArm");

        stopAllMotors();

        telemetry.addData("Motor Initialization Complete", "");

        rangeSensorB = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorB");
        rangeSensorL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorL");

        servoWinchArm.setPosition(servoWinchArmInitPos);

        telemetry.addData("Servo Initialization Complete", "");

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("IMU Initialization Complete", "");

        telemetry.addData("Initialization Complete", "");
        telemetry.update();

        motorLiftDown1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        motorLiftDown1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftDown2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLiftDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftDown2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
        motorExtend.setPower(0);
    }

    public void delatch() throws InterruptedException{
        resetEncoders();

        motorLiftDown1.setPower(.8);
        motorLiftDown2.setPower(.8);

        while (motorLiftDown1.getCurrentPosition() < 3500 && opModeIsActive()) {
        }

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
    }

    public void lowerLift() throws InterruptedException {
        motorLiftDown1.setPower(-.75);
        motorLiftDown2.setPower(-.75);

        while (motorLiftDown1.getCurrentPosition() > 50 && opModeIsActive()) {
        }

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
    }

    public void Pturn(double angle, int msTimeout) {
        double kP = .6/90;
        double minSpeed = .30;
        double maxSpeed = 1;
        time.reset();

        while (Math.abs(imu.getTrueDiff(angle)) > .5 && time.milliseconds() < msTimeout && opModeIsActive()) {
            double angleError = imu.getTrueDiff(angle);

            double PIDchange = kP * angleError;

            if (PIDchange > 0 && PIDchange < minSpeed)
                PIDchange = minSpeed;
            else if (PIDchange < 0 && PIDchange > -minSpeed)
                PIDchange = -minSpeed;

            motorBL.setPower(Range.clip(-PIDchange * left, -maxSpeed, maxSpeed));
            motorFL.setPower(Range.clip(-PIDchange * left, -maxSpeed, maxSpeed));
            motorBR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));
            motorFR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));

            telemetry.addData("angleError: ", angleError);
            telemetry.addData("PIDCHANGE: ", PIDchange);
            telemetry.update();
        }
        stopMotors();
    }

    public void PturnRight(double angle, int msTimeout) {
        double kP = .6/90;
        double minSpeed = .7;
        double maxSpeed = 1;
        time.reset();

        while (Math.abs(imu.getTrueDiff(angle)) > .5 && time.milliseconds() < msTimeout && opModeIsActive()) {
            double angleError = imu.getTrueDiff(angle);

            double PIDchange = kP * angleError;

            if (PIDchange > 0 && PIDchange < minSpeed)
                PIDchange = minSpeed;
            else if (PIDchange < 0 && PIDchange > -minSpeed)
                PIDchange = -minSpeed;

            motorBR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));
            motorFR.setPower(Range.clip(PIDchange, -maxSpeed, maxSpeed));

            telemetry.addData("angleError: ", angleError);
            telemetry.addData("PIDCHANGE: ", PIDchange);
            telemetry.update();
        }
        stopMotors();
    }

    public void moveTimeP(double msTime, double power, double angle) throws InterruptedException {
        time.reset();

        double kPangle = 1.0/90.0;

        while (time.milliseconds() < msTime) {
            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            //if (power > 0) {
            motorBL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
            motorFL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
            motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //} else if (power < 0) {
            //  motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            // motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            //motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            //}
        }
        stopMotors();
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }

    public void moveToDistP(double inches, double angle, double timeout) {
        double kPdist = .03;
        double kPangle = .9/90.0;
        // MIGHT NEED TO BE RETUNED
        double minDrive = .15;
        double maxDrive = .5;

        time.reset();
        while ((Math.abs(getDistB() - inches) > .25 || imu.getTrueDiff(angle) > .5) && opModeIsActive() && time.milliseconds() < timeout) {

            double distError = inches - getDistB();
            double PIDchangeDist = -Range.clip(-kPdist * distError, -maxDrive, maxDrive);

            if (PIDchangeDist < minDrive && PIDchangeDist > 0) {
                PIDchangeDist = minDrive;
            } else if (PIDchangeDist > -minDrive && PIDchangeDist < 0) {
                PIDchangeDist = -minDrive;
            }

            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            motorBL.setPower(Range.clip((PIDchangeDist - PIDchangeAngle) * left, -1, 1));
            motorFL.setPower(Range.clip((PIDchangeDist - PIDchangeAngle) * left, -1, 1));
            motorBR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
        }
        stopMotors();
    }


    public void moveToEncoder(double encoder, double power, double angle) {
        resetEncoders();

        double kPangle = 3.0/90.0;              // MIGHT NEED TO BE RETUNED

        if (encoder > 0) {
            while (motorBL.getCurrentPosition() < encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;

                motorBL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
                motorFL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorBL.getCurrentPosition() > encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;

                motorBL.setPower(Range.clip((-power - PIDchangeAngle) * left, -1, 1));
                motorFL.setPower(Range.clip((-power - PIDchangeAngle) * left, -1, 1));
                motorBR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
            }
        }
        stopMotors();
    }

    public void moveToEncoderT(double encoder, double power, double angle, double msTimeout) {

        resetEncoders();

        double kPangle = 3.0/90.0;              // MIGHT NEED TO BE RETUNED
        time.reset();
        if (encoder > 0) {
            while (motorBL.getCurrentPosition() < encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
                motorFL.setPower(Range.clip((power - PIDchangeAngle) * left, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorBL.getCurrentPosition() > encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip((-power - PIDchangeAngle) * left, -1, 1));
                motorFL.setPower(Range.clip((-power - PIDchangeAngle) * left, -1, 1));
                motorBR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
            }
        }
        stopMotors();
    }

    //˯˯ Sets motor power to zero
    public void stopDriveMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    //˯˯ Sets motor and winch power to zero
    public void stopAllMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);

       // motorWinchDown.setPower(0);
        //motorWinchUp.setPower(0);
    }




    public void depositMarker() throws InterruptedException {
        //this actually works now
        servoWinchArm.setPosition(servoWinchArmDepositPos);
        sleep(1000);
        servoWinchArm.setPosition(servoWinchArmInitPos);
    }

    /* :)
    public void getJewelColor() {
        //jewel camera init
        telemetry.addLine("JewelCamera initialization started");
        telemetry.update();

        setCameraDownsampling(2);

        telemetry.addLine("Wait for camera to finish initializing!");
        telemetry.update();

        startCamera();  // can take a while.

        sleep(50);

        telemetry.addLine("Camera ready!");
        telemetry.update();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        int numPics = 0;
        int redValue = 0;
        int blueValue = 0;
        int numFailLoops = 0;

        while (time.seconds() < 2 && opModeIsActive()) {
            if (imageReady()) { // only do this if an image has been returned from the camera

                numPics++;

                // get image, rotated so (0,0) is in the bottom left of the preview window
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                for (int x = (int) (.8 * rgbImage.getWidth()); x < rgbImage.getWidth(); x++) {
                    for (int y = 0; y < (int) (.25 * rgbImage.getHeight()); y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                    }
                }
            } else {
                numFailLoops++;
            }

            sleep(10);
        }

        boolean jewelIsRed = redValue > blueValue;

        stopCamera();

        telemetry.addData("Is Jewel Red?", jewelIsRed);

        telemetry.addData("numPics: ", numPics);
        telemetry.addData("numFailLoops: ", numFailLoops);
        telemetry.addData("red blue: ", redValue + "    " + blueValue);
    } */


    public double getDistB() {
        double dist = rangeSensorB.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist)) && opModeIsActive()) {
            dist = rangeSensorB.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }

    public double getDistL() {
        double dist = rangeSensorL.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist)) && opModeIsActive()) {
            dist = rangeSensorL.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }


    public void moveToLineP(double yIntercept, double angle, double timeout) {
        double kPdist = .0105;
        double kPangle = .9/90.0;
                                        //ONLY MODIFY THESE PARAMETERS, NOTHING BELOW THEM
        double minDrive = .135;
        double maxDrive = .5;

        time.reset();

        while ((Math.abs(getDistB() - getDistL() - yIntercept) > .25 && opModeIsActive() && time.milliseconds() < timeout)) { // lol xd

            double distError = getDistB() - getDistL() - yIntercept;
            double PIDchangeDist = Range.clip(-kPdist * distError, -maxDrive, maxDrive);

            telemetry.addData("distError: ", distError);
            telemetry.addData("PIDchangeDist: ", PIDchangeDist);
            telemetry.update();
            if (PIDchangeDist < minDrive && PIDchangeDist > 0) {
                PIDchangeDist = minDrive;
            } else if (PIDchangeDist > -minDrive && PIDchangeDist < 0) {
                PIDchangeDist = -minDrive;
            }

            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            motorBL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorBR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
        }
        stopDriveMotors();
    }

    public void resetEncoders() {
        //honestly not sure if this works but it seems like it should
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();         // don't worry about this i'm told it does something

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void getBlock() throws InterruptedException {
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("tfod is null? ", tfod == null);
        tensorflowInfo += "tfod is null? " + tfod == null + "\n";

        List<Recognition> recognitions = null;
        boolean twoObjectsFound = false;

        while (recognitions == null || !twoObjectsFound) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                recognitions = tfod.getUpdatedRecognitions();
                if (recognitions != null) {
                    telemetry.addData("# Object Detected", recognitions.size());
                    tensorflowInfo += "# Object Detected " + recognitions.size() + "\n";

                    int min1X;
                    int min2X;
                    String min1Label;
                    String min2Label;

                    if (recognitions.size() >= 2) {
                        twoObjectsFound = true;
                        Collections.sort(recognitions, new Comparator<Recognition>() {
                            @Override
                            public int compare(Recognition recognition, Recognition t1) {
                                if (recognition.getBottom() > t1.getBottom())
                                    return -1;
                                return 1;
                            }

                        });

                        min1Label = recognitions.get(0).getLabel();
                        min1X = (int) recognitions.get(0).getLeft();

                        min2Label = recognitions.get(1).getLabel();
                        min2X = (int) recognitions.get(1).getLeft();

                        if (min1Label.equals(LABEL_SILVER_MINERAL) && min2Label.equals(LABEL_SILVER_MINERAL)) {
                            blockPos = 'R';
                        } else {
                            if (min1Label.equals(LABEL_GOLD_MINERAL)) {
                                blockPos = min1X < min2X ? 'L' : 'C';
                            } else if (min2Label.equals(LABEL_GOLD_MINERAL)) {
                                blockPos = min2X < min1X ? 'L' : 'C';
                            }
                        }
                        telemetry.addData("blockPos: ", blockPos);

                    } else {
                        //well shit
                        telemetry.addLine("Less than 2 blocks found");
                        tensorflowInfo += "Less than 2 blocks found" + "\n";
                    }
                    telemetry.update();
                }
            }
        }
    }

    public int yellowValOfBox(Bitmap bmp, BoundingBox bb) {
        int ySum = 0;

        //scans bounding box
        for (int r = bb.startRow; r < bb.endRow && opModeIsActive(); r++) {
            for (int c = bb.startCol;  c < bb.endCol && opModeIsActive(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int B = color & 0xff;
                int yellow = Math.min(R, G) - B  > 0 ? Math.min(R, G) - B : 0;
                ySum += yellow;
            }
        }

        //scans entire bitmap
        /*or (int r = 0; r < bmp.getHeight(); r++) {
            for (int c = 0;  c < bmp.getWidth(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int yellow = Math.min(R, G);
                ySum += yellow;
            }
        }*/

        return ySum;
    }

    public Bitmap takePic() throws InterruptedException {
        File dir = Environment.getExternalStorageDirectory();
        File[] files = dir.listFiles();
        int highestFileNum = -1;
        for (File f : files) {
            if (f.getName().contains("BoTest_pic_")) {
                highestFileNum = Math.max(highestFileNum, Integer.parseInt(f.getName().replaceAll("\\D", "")));
            }
        }
        int currFileNum = highestFileNum + 1;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        telemetry.addData("Num images in frame", "" + frame.getNumImages());
        telemetry.update();
        //for (int i = 0; i < frame.getNumImages() && opModeIsActive(); i++) {
        Image image = frame.getImage(0);

        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
        Bitmap bmp = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bmp.copyPixelsFromBuffer(image.getPixels());
        try {
            File sdCard = Environment.getExternalStorageDirectory();
            telemetry.addLine(sdCard.getAbsolutePath());
            telemetry.update();
            //File dir = new File(sdCard.getAbsolutePath() + "/dir1");
            //dir.mkdirs();

            File file = new File(sdCard, "BoTest_pic_" + currFileNum++ + ".png");

            FileOutputStream fos = new FileOutputStream(file);

            bmp = getResizedBitmap(bmp, 78, 59);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);

        } catch (FileNotFoundException ex) {
            telemetry.addLine(ex.toString());
            telemetry.update();
        }
        //ByteBuffer byteBuffer = image.getPixels();
                    /*if (frameBuffer == null) {
                        frameBuffer = new byte[byteBuffer.capacity()];
                    }
                    byteBuffer.get(frameBuffer);
                    if (this.frame == null) {
                        this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);

                        if (overlayView != null) {
                            overlayView.setImageSize(imageWidth, imageHeight);
                        }
                    }
                    this.frame.put(0, 0, frameBuffer);

                    Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                    if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                        Core.flip(this.frame, this.frame, 1);
                    }

                    onFrame(this.frame, vuforiaFrame.getTimeStamp());*/
        return bmp;
    }

    public Bitmap getResizedBitmap(Bitmap bm, int newWidth, int newHeight) {
        int width = bm.getWidth();
        int height = bm.getHeight();
        float scaleWidth = ((float) newWidth) / width;
        float scaleHeight = ((float) newHeight) / height;
        // CREATE A MATRIX FOR THE MANIPULATION
        Matrix matrix = new Matrix();
        // RESIZE THE BIT MAP
        matrix.postScale(scaleWidth, scaleHeight);

        // "RECREATE" THE NEW BITMAP
        Bitmap resizedBitmap = Bitmap.createBitmap(
                bm, 0, 0, width, height, matrix, false);
        bm.recycle();
        return resizedBitmap;
    }

    public void getBlockAcc() throws InterruptedException {
        Bitmap bitmap = takePic();

        int N = 4;
        BoundingBox block = null;
        BoundingBox ball1 = null;
        BoundingBox ball2 = null;
        int yellow = -1;
        int white = -1;
        int white2 = -1;
        for (int r = 0; r < bitmap.getHeight() - N; r++) {
            for (int c = 0; c < bitmap.getWidth() - N; c++) {
                BoundingBox testBox = new BoundingBox(r, c, r+4, c+4);
                int testYellow = yellowValOfBox(bitmap, testBox);
                int testWhite = whiteValOfBox(bitmap, testBox);
                if (testYellow > yellow) {
                    yellow = testYellow;
                    block = testBox;
                }
                if (testWhite > white) {
                    white = testWhite;
                    ball1 = testBox;
                }
                else if (testWhite > white2 && Math.abs(testBox.startCol - ball1.startCol) > 10) {
                    white2 = testWhite;
                    ball2 = testBox;
                }
            }
        }

        if (block.startCol < ball1.startCol && block.startCol < ball2.startCol)
            blockPos = 'L';
        else if (block.startCol > ball1.startCol && block.startCol > ball2.startCol)
            blockPos = 'R';
        else
            blockPos = 'C';
    }

    //calculates how white a region is; not currently used in detection
    public int whiteValOfBox(Bitmap bmp, BoundingBox bb) {
        int whiteSum = 0;

        for (int r = bb.startRow; r < bb.endRow && opModeIsActive(); r++) {
            for (int c = bb.startCol;  c < bb.endCol && opModeIsActive(); c++) {
                int color = bmp.getPixel(c, r);
                int R = (color >> 16) & 0xff;
                int G = (color >>  8) & 0xff;
                int B = color & 0xff;
                int white = Math.min(R, Math.min(G, B));
                whiteSum += white;
            }
        }

        return whiteSum;
    }

    public Bitmap saveBox(Bitmap bmp, BoundingBox bb) throws InterruptedException{
        File dir = Environment.getExternalStorageDirectory();
        File[] files = dir.listFiles();
        int highestFileNum = -1;
        for (File f : files) {
            if (f.getName().contains("pixel_match_")) {
                highestFileNum = Math.max(highestFileNum, Integer.parseInt(f.getName().replaceAll("\\D", "")));
            }
        }
        int currFileNum = highestFileNum + 1;

        try {
            File sdCard = Environment.getExternalStorageDirectory();
            telemetry.addLine(sdCard.getAbsolutePath());
            telemetry.update();
            //File dir = new File(sdCard.getAbsolutePath() + "/dir1");
            //dir.mkdirs();

            File file = new File(sdCard, "pixel_match_" + currFileNum++ + ".png");

            FileOutputStream fos = new FileOutputStream(file);

            bmp = getResizedBitmap(bmp, 78, 59);
            Bitmap drawnBmp = bmp.copy(Bitmap.Config.RGB_565, true);

            Canvas canvas = new Canvas(drawnBmp);
            Paint p = new Paint();
            p.setStyle(Paint.Style.FILL_AND_STROKE);
            p.setAntiAlias(true);
            p.setFilterBitmap(true);
            p.setDither(true);
            p.setColor(Color.RED);

            canvas.drawLine(bb.startCol, bb.startRow, bb.startCol, bb.endRow, p);//left
            canvas.drawLine(bb.startCol, bb.startRow, bb.endCol, bb.startRow, p);//top
            canvas.drawLine(bb.endCol, bb.startRow, bb.endCol, bb.endRow, p);//right
            canvas.drawLine(bb.startCol, bb.endRow, bb.endCol, bb.endRow, p);//bottom

// rect ...
//canvas.drawRect(/*all of my end coordinates*/, p);

            //ImageView iView = (ImageView)findViewById(R.id.imageViewPreview);
            //iView.setImageBitmap(drawnBmp);
            //iView.draw(canvas);

            drawnBmp.compress(Bitmap.CompressFormat.PNG, 100, fos);

        } catch (FileNotFoundException ex) {
            telemetry.addLine(ex.toString());
            telemetry.update();
        }

        return bmp;
    }

    protected class BoundingBox {
        int startRow;
        int endRow;
        int startCol;
        int endCol;

        public BoundingBox(int startRow, int startCol, int endRow, int endCol) {
            this.startRow = startRow;
            this.endRow = endRow;
            this.startCol = startCol;
            this.endCol = endCol;
        }
    }


}