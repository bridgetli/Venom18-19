package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    DcMotor motorExtend;
    DcMotor motorLiftDown1;
    DcMotor motorLiftDown2;
    DcMotor motorManip;

    IMU imu;

    //winch motors???
    //DcMotor motorWinchUp;
    //DcMotor motorWinchDown;
    ModernRoboticsI2cRangeSensor rangeSensorB;
    ModernRoboticsI2cRangeSensor rangeSensorL;

    final double winchDownPower = .5;
    final double winchUpPower = .5;

    Servo servoWinchArm;
    Servo servoGate;
    Servo servoBasket;

    CRServo servoLeftManip;
    CRServo servoRightManip;

    final double servoWinchArmDownPos = .5;
    final double servoWinchArmUpPos = 1;

    //Servo servoMarker;

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

        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorLiftDown1 = hardwareMap.dcMotor.get("motorLiftDown1");
        motorLiftDown2 = hardwareMap.dcMotor.get("motorLiftDown2");

        motorManip = hardwareMap.dcMotor.get("motorManip");

        rangeSensorB = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorB");
        rangeSensorL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorL");

       // motorWinchUp = hardwareMap.dcMotor.get("motorWinchUp");
        //motorWinchDown = hardwareMap.dcMotor.get("motorWinchDown");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //motorWinchDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWinchUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWinchDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLiftDown1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorWinchUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLiftDown1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLiftDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftDown2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLiftDown1.setPower(0);

        stopAllMotors();

        telemetry.addData("Motor Initialization Complete", "");

        //servoMarker = hardwareMap.servo.get("servoMarker");
        //servoMarker.setPosition(0);

        servoWinchArm = hardwareMap.servo.get("servoWinchArm");
        servoWinchArm.setPosition(servoWinchArmDownPos);

        servoBasket = hardwareMap.servo.get("servoBasket");
        servoGate = hardwareMap.servo.get("servoGate");
        servoBasket.setPosition(0);
        servoGate.setPosition(.5);

        servoLeftManip = hardwareMap.crservo.get("servoLeftManip");
        servoRightManip = hardwareMap.crservo.get("servoRightManip");


        telemetry.addData("Servo Initialization Complete", "");



        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("IMU Initialization Complete", "");

        //initVuforia();
        //initTfod();

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

        motorLiftDown1.setPower(0);
        motorLiftDown2.setPower(0);
        motorExtend.setPower(0);
    }

    public void setLeftMotors(double left){
        motorFL.setPower(left);
        motorBL.setPower(left);
    }

    public void setRightMotors(double right){
        motorFR.setPower(right);
        motorBR.setPower(right);
    }


    public double rightABSMotorVal(double joyStickVal) {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorBR.getPower() + maxJump) {
            return Range.clip(motorBR.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBR.getPower() - maxJump) {
            return Range.clip(motorBR.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }

    public double leftABSMotorVal(double joyStickVal) {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorBL.getPower() + maxJump) {
            return Range.clip(motorBL.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBL.getPower() - maxJump) {
            return Range.clip(motorBL.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }

    public double getDistB() {
        double dist = rangeSensorB.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist))) {
            dist = rangeSensorB.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }

    public double getDistL() {
        double dist = rangeSensorL.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist))) {
            dist = rangeSensorL.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }

    //void initVuforia() {}
    //void initTfod() {} (make sure to put device validation in method)
    //:)
}