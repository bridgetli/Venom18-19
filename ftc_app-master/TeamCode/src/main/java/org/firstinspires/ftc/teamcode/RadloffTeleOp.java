package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="RadloffOPOpMode", group="OpMode")
public class RadloffTeleOp extends OpMode {
        private DcMotor L;
        private DcMotor R;

        public void init() {
            L = hardwareMap.get(DcMotor.class, "L");
            R = hardwareMap.get(DcMotor.class, "R");
        }

        public void loop() {
            if (Math.abs(gamepad1.right_stick_y) > 0.1){
                R.setPower(-gamepad1.right_stick_y);
            }
            else{
                stopMotors();
            }

            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                L.setPower(gamepad1.left_stick_y);
            }
            else{
                stopMotors();
            }
        }

        public void stopMotors() {
            L.setPower(0);
            R.setPower(0);
        }
}