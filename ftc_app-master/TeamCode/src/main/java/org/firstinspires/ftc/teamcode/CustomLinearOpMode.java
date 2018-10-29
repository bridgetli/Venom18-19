package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import for_camera_opmodes.LinearOpModeCamera;

import static android.graphics.Color.blue;
import static android.graphics.Color.red;

public class CustomLinearOpMode extends LinearOpMode {
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


    ModernRoboticsI2cRangeSensor rangeSensor;



    final double winchDownPower = .5;
    final double winchUpPower = .5;

    Servo servoMarker;

    final double servoMarkerStartPos = 1;
    final double servoMarkerEndPos = 0;

    IMU imu;

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

        stopAllMotors();

        telemetry.addData("Motor Initialization Complete", "");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");


        servoMarker = hardwareMap.servo.get("servoMarker");
        servoMarker.setPosition(0);

        telemetry.addData("Servo Initialization Complete", "");

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("IMU Initialization Complete", "");

        telemetry.addData("Initialization Complete", "");

        //waitForStart();
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
            while (yaw > angle && opModeIsActive()){
                turnLeft();
            }
        }
        stopDriveMotors();
    }

    public void driveForward (){
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);
    }

    public void release() throws InterruptedException{
        //lower the robot??
        Thread.sleep(400); // we might wanna PID this
       // motorWinchDown.setPower(winchDownPower);
        // we might wanna PID this
        try {
            Thread.sleep(400);
        } catch (Exception e) {
            stop();
        }

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
    public void depositMarker() {
        // deposits the marker in the thing
    }

    public String getBlockLocation() {
        return "CENTER";
    }

    public void goForward(double distance){
        // goes forward a certain distance after we add the sensor in
        // distance is in inches

        double oldDist = getDist();
        double newDist = getDist();
        while(Math.abs(oldDist - newDist) < distance && opModeIsActive()) {
            driveForward();
            newDist = getDist();
            telemetry.addData("Stuck in the loop", "");
        }
        stopDriveMotors();
    }
    public double getDist() {
        double dist = rangeSensor.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist)) && opModeIsActive()) {
            dist = rangeSensor.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }

    public void moveToDistance(double dist) {
        while(getDist() > dist && opModeIsActive()) {
            driveForward();
        }
        while(getDist() < dist && opModeIsActive()) {
            driveBackward();
        }
        stopDriveMotors();
    }
}