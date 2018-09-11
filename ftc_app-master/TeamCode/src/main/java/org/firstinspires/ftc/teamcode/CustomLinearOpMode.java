package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class CustomLinearOpMode extends LinearOpMode{
    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    //speed
    double speed = 0.5;

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


    @Override
    public void runOpMode() {

    }


    // initzialization method
    public void initizialize() {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

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

    //˯˯ Turn method (no PID loop)
    public void Turn(double angle)
    {
        double yaw = imu.getYaw();
        if (angle > yaw) {
            while (yaw < angle) {
                TurnRight();
            }
        }
        else if (angle < yaw) {
            while (yaw > angle){
                TurnLeft();
            }
        }
    }
    public void TurnRight {
        motorFL.setPower(-speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
    }

    public void TurnLeft {
        motorFL.setPower(speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
    }

    public void release() throws InterruptedException{
        //lower the robot??
        motorWinchDown.setPower(winchDownPower);
        Thread.sleep(400); // we might wanna PID this
    }
}
