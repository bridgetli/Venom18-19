package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Created by Ryan Branstetter on 9/11/18
 * Program contains the basics for the robot
 * like the hardware mapping and stuff
 */

public class BasicRobot extends OpMode{
    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    //winch motors???
    DcMotor motorWinchUp;
    DcMotor motorWinchDown;


    final double winchDownPower = .5;
    final double winchUpPower = .5;

    Servo servoMarker;

    final double servoMarkerStartPos = 1;
    final double servoMarkerEndPos = 0;

    IMU imu;

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

        motorWinchUp = hardwareMap.dcMotor.get("motorWinchUp");
        motorWinchDown = hardwareMap.dcMotor.get("motorWinchDown");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWinchUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWinchDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWinchUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWinchDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        stopAllMotors();

        telemetry.addData("Motor Initialization Complete", "");

        servoMarker = hardwareMap.servo.get("servoMarker");
        servoMarker.setPosition(0);

        telemetry.addData("Servo Initialization Complete", "");

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("IMU Initialization Complete", "");

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

        motorWinchDown.setPower(0);
        motorWinchUp.setPower(0);
    }
    public void release() throws InterruptedException{
        //lower the robot??
        motorWinchDown.setPower(winchDownPower);
        Thread.sleep(400); // we might wanna PID this
    }



}
