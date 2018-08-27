package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="RookieCampTele", group="OpMode")



public class RookieCampTele extends OpMode {


    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    IMU imu;


    double LD = 0;
    double LU = 1;
    double RD = 0;
    double RU = 1;


    Servo Left;
    Servo Right;


    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Left = hardwareMap.servo.get("Left");
        Right = hardwareMap.servo.get("Right");

        Left.setPosition(LU);
        Right.setPosition(RU);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);


    }


    public void loop() {

        if (Math.abs(gamepad1.right_stick_y) > 0.1){
            FL.setPower(gamepad1.right_stick_y);
            BL.setPower(gamepad1.right_stick_y);
        }
        else{
            stopMotors();
        }

        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            FR.setPower(gamepad1.left_stick_y);
            BR.setPower(gamepad1.left_stick_y);
        }
        else{
            stopMotors();
        }

        if (Math.abs(gamepad1.left_trigger) > 0.1) {
            Left.setPosition(LD);
            Right.setPosition(RD);
        }

        if (Math.abs(gamepad1.right_trigger) > 0.1) {
            Left.setPosition(LU);
            Right.setPosition(RU);
        }

    }

    public void stopMotors() {

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);


    }


}