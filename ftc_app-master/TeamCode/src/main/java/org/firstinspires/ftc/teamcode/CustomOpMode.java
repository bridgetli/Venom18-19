package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CustomOpMode extends OpMode{
    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorWL; //carabiner
    DcMotor motorWR; //release

    IMU imu;

    //winch motors???
    //DcMotor motorWinchUp;
    //DcMotor motorWinchDown;
    ModernRoboticsI2cRangeSensor rangeSensor;

    final double winchDownPower = .5;
    final double winchUpPower = .5;

    Servo servoWinchArm;

    final double servoWinchArmDownPos = .2;
    final double servoWinchArmUpPos = .42;

    //Servo servoMarker;

    final double servoMarkerStartPos = 1;
    final double servoMarkerEndPos = 0;

    //IMU imu;

    //just had to put these to run the code dw about it
    public void init() {

    }
    public void loop() {
        
    }


    // initzialization method
    public void initizialize() {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorWL = hardwareMap.dcMotor.get("motorWL");
        motorWR = hardwareMap.dcMotor.get("motorWR");

       // motorWinchUp = hardwareMap.dcMotor.get("motorWinchUp");
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

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        stopAllMotors();

        telemetry.addData("Motor Initialization Complete", "");

        //servoMarker = hardwareMap.servo.get("servoMarker");
        //servoMarker.setPosition(0);

        servoWinchArm = hardwareMap.servo.get("servoWinchArm");
        servoWinchArm.setPosition(servoWinchArmDownPos);


        telemetry.addData("Servo Initialization Complete", "");



        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("IMU Initialization Complete", "");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");;

        telemetry.addData("Initialization Complete", "");


    }

    public void stopDriveMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    public void stopAllMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);

        //motorWinchDown.setPower(0);
        //motorWinchUp.setPower(0);
    }

    public void setLeftMotors(double left){
        motorFL.setPower(left);
        motorBL.setPower(left);
    }

    public void setRightMotors(double right){
        motorFR.setPower(right);
        motorBR.setPower(right);
    }
    
    public void release() throws InterruptedException {
        //lower the robot??
        //motorWinchDown.setPower(winchDownPower);
        Thread.sleep(400); // we might wanna PID this
    }

    // copy pasted ABS from last year
    public double rightABSMotorVal(double joyStickVal) {
        /*if (Math.abs(joyStickVal - motorBL.getPower()) < 1) {
            return joyStickVal;
        }*/
        double c = .25;
        if (joyStickVal >= motorBR.getPower()) {
            return Range.clip(motorBR.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBR.getPower()) {
            return Range.clip(motorBR.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }
    public double leftABSMotorVal(double joyStickVal) {
        double c = .25;
        if (joyStickVal >= motorBL.getPower()) {
            return Range.clip(motorBL.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBL.getPower()) {
            return Range.clip(motorBL.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }
    public double getDist() {
        return rangeSensor.getDistance(DistanceUnit.INCH);
    }


}