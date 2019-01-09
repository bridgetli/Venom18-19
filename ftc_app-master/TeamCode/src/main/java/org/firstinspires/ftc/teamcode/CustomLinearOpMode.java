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

import java.util.List;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

import static android.graphics.Color.blue;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class CustomLinearOpMode extends LinearOpMode {

    protected static final String VUFORIA_KEY = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    VuforiaLocalizer vuforia;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    //speed
    double speed = 0.5;

    //winch motors???
    //DcMotor motorWinchUp;
    //DcMotor motorWinchDown;


    ModernRoboticsI2cRangeSensor rangeSensorB;
    ModernRoboticsI2cRangeSensor rangeSensorL;

    Servo servoWinchArm;
    final double servoWinchArmInitPos = .1;
    final double servoWinchArmDepositPos = .79;

    final double winchDownPower = .5;
    final double winchUpPower = .5;

    //Servo servoMarker;

    final double servoMarkerStartPos = 1;
    final double servoMarkerEndPos = 0;

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

        //Vuforia and Tensorflow init (This only works on the Motorola)
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
            initTfod();
        else
            telemetry.addLine("Please use the Motorolas if you want to use Tensorflow");
        telemetry.addData("Vuforia and Tensorflow Initialization Complete", "");


        telemetry.addData("Initialization Complete", "");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        telemetry.addLine("Vuforia initialization complete");
        //waitForStart();
    }

    public void Pturn(double angle, int msTimeout) {
        double kP = .35/90;
        double minSpeed = .2;
        double maxSpeed = .6;
        time.reset();

        while (Math.abs(imu.getTrueDiff(angle)) > .5 && time.milliseconds() < msTimeout && opModeIsActive()) {
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

    public void moveTimeP(double msTime, double power, double angle) throws InterruptedException {
        time.reset();

        double kPangle = 1.0/90.0;

        while (time.milliseconds() < msTime) {
            double angleError = imu.getTrueDiff(angle);
            double PIDchangeAngle = kPangle * angleError;

            //if (power > 0) {
            motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
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

            motorBL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorFL.setPower(Range.clip(PIDchangeDist - PIDchangeAngle, -1, 1));
            motorBR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
            motorFR.setPower(Range.clip(PIDchangeDist + PIDchangeAngle, -1, 1));
        }
        stopMotors();
    }


    public void moveToEncoder(double encoder, double power, double angle) {
        resetEncoders();

        double kPangle = 1.0/90.0;              // MIGHT NEED TO BE RETUNED

        if (encoder > 0) {
            while (motorFL.getCurrentPosition() < encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorFL.getCurrentPosition() > encoder && opModeIsActive()) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
            }
        }
        stopMotors();
    }

    public void moveToEncoderT(double encoder, double power, double angle, double msTimeout) {

        resetEncoders();

        double kPangle = 1.0/90.0;              // MIGHT NEED TO BE RETUNED
        time.reset();
        if (encoder > 0) {
            while (motorFL.getCurrentPosition() < encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(power - PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(power + PIDchangeAngle, -1, 1));
            }
        }
        else {
            while (motorFL.getCurrentPosition() > encoder && opModeIsActive() && time.milliseconds() < msTimeout) {

                double angleError = imu.getTrueDiff(angle);
                double PIDchangeAngle = kPangle * angleError;


                motorBL.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorFL.setPower(Range.clip(-power + PIDchangeAngle, -1, 1));
                motorBR.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
                motorFR.setPower(Range.clip(-power - PIDchangeAngle, -1, 1));
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

    //˯˯ Sets motors to turn right when called in the Turn method
    public void turnRight() {
        motorFL.setPower(-speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
    }

    //˯˯ Sets motors to turn left when called in the Turn method
    public void turnLeft() {
        motorFL.setPower(speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
    }

    //˯˯ Turn method (no PID loop)
    public void turn(double angle)
    {
        double yaw = imu.getYaw();
        if (angle > yaw) {
            while (yaw < angle && opModeIsActive()) {
                turnRight();
            }
        }
        else if (angle < yaw) {
            while (yaw > angle && opModeIsActive()) {
                turnLeft();
            }
        }
        stopDriveMotors();
    }

    public void driveForward() {
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);
    }

    public void depositMarker() throws InterruptedException {
        //this actually works now
        servoWinchArm.setPosition(servoWinchArmDepositPos);
        sleep(1500);
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

    public void driveBackward() {
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
    }

    public String getBlockLocation() {
        return "CENTER";
    }

    public void goForward(double distance){
        // goes forward a certain distance after we add the sensor in
        // distance is in inches

        double oldDist = getDistB();
        double newDist = getDistB();
        while(Math.abs(oldDist - newDist) > distance && opModeIsActive()) {
            driveForward();
            newDist = getDistB();
            telemetry.addData("Stuck in the loop", "");
        }
        stopDriveMotors();
    }
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

    public void moveToDistance(double dist) {
        while(getDistB() > dist && opModeIsActive()) {
            driveForward();
        }
        while(getDistB() < dist && opModeIsActive()) {
            driveBackward();
        }
        stopDriveMotors();
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
    
    public void moveToLineP(double yIntercept, double angle, double timeout) { //y-int is 64?
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

    //TODO replace getBlock with this method; also, make sure this is up to date with TFTest
    public void getGoldCubePos() {
        char pos = 'C';
        int numAttempts = 5; //adjust if necessary
        if (tfod != null)
            tfod.activate();
        for(int attempts = 0; attempts < numAttempts; attempts++) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            } else {
                                silverMineral2X = (int) recognition.getTop();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                pos = 'L';
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                pos = 'R';
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                pos = 'C';
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null)
            tfod.shutdown();
        blockPos = pos;
    }

    public void getBlock() throws InterruptedException {
        // Retrieves the location of the yellow block and saves it in blockPos

        blockPos = 'C'; //default case

        Bitmap bitmap = takePic();


        // basic brute force counter
        int startRow = 14;                  // VERTICAL DIMENSIONS
        int endRow = 23;
        // WILL NEED TO BE TUNED AGAIN WITH NEW PHONE LOCATION
        int leftScol = 13;                  // HORIZONTAL DIMENSIONS
        int leftEcol = 22;
        int centerScol = 35;
        int centerEcol = 44;
        int rightScol = 56;
        int rightEcol = 65;

        BoundingBox left = new BoundingBox(startRow, leftScol, endRow, leftEcol);    //look at images taken from consistent
        BoundingBox center = new BoundingBox(startRow, centerScol, endRow, centerEcol);  //spot in auto and get pixel range
        BoundingBox right = new BoundingBox(startRow, rightScol, endRow, rightEcol);   //of left center and right

        if (yellowValOfBox(bitmap, left) > yellowValOfBox(bitmap, center)) {
            blockPos = 'L';
            if (yellowValOfBox(bitmap, right) > yellowValOfBox(bitmap, left))
                blockPos = 'R';
        }
        else if (yellowValOfBox(bitmap, right) > yellowValOfBox(bitmap, center))
            blockPos = 'R';

        //saveBox(bitmap, left);
        //saveBox(bitmap.copy(Bitmap.Config.RGB_565, true), center);
        //saveBox(bitmap.copy(Bitmap.Config.RGB_565, true), right);

        // multi location pixel scanner (better but much slower)

        /*int N = 4; // the approx height and width of an object

        for (int r = 0; r < */
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
                int yellow = Math.min(R, G) - B / 2 > 0 ? Math.min(R, G) - B / 2 : 0;
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