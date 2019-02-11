package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="mainTele", group="TeleOp")
public class mainTele extends CustomOpMode {
    double motorScale = -1;

    boolean rT = false;
    boolean lT = false;

    public void init() {
        initizialize();
    }
    public void loop() {

        double yL = gamepad1.left_stick_y;
        double yR = gamepad1.right_stick_y;

        /*if (imu.getPitch() < -39) {
            motorFL.setPower(1);
            motorBL.setPower(1);
            motorFR.setPower(1);
            motorBR.setPower(1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {

            }
        } else if (imu.getPitch() > 39) {
            motorFL.setPower(-1);
            motorBL.setPower(-1);
            motorFR.setPower(-1);
            motorBR.setPower(-1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {

            }
        }*/

        if (Math.abs(yL) > .2) {
            //motorBL.setPower(leftABSMotorVal(yR) * motorScale);
            //motorFL.setPower(leftABSMotorVal(yR) * motorScale);
            if (motorScale == -1) {
                motorBL.setPower(yL * motorScale);
                motorFL.setPower(yL * motorScale);
            } else {
                motorBR.setPower(yL * motorScale);
                motorFR.setPower(yL * motorScale);
            }
        } else {
            motorBL.setPower(0);
            motorFL.setPower(0);
        }

        if(Math.abs(yR) > .2) {
            //motorBR.setPower(rightABSMotorVal(yL) * motorScale);
            //motorFR.setPower(rightABSMotorVal(yL) * motorScale);
            if (motorScale == -1) {
                motorBR.setPower(yR * motorScale);
                motorFR.setPower(yR * motorScale);
            } else {
                motorBL.setPower(yR * motorScale);
                motorFL.setPower(yR * motorScale);
            }
        } else {
            motorBR.setPower(0);
            motorFR.setPower(0);
        }

        if (gamepad1.right_trigger > .1) {
            motorScale = motorScale == -1 ? 1 : -1;
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {

            }

        }

        if (gamepad2.dpad_down) {
            motorLiftDown1.setPower(-1);
            motorLiftDown2.setPower(-1);
            //motorExtend.setPower(.5);
        } else if (gamepad2.dpad_up){
            motorLiftDown1.setPower(1);
            motorLiftDown2.setPower(1);
            //motorExtend.setPower(-.5);
        } else {
            motorLiftDown1.setPower(0);
            motorLiftDown2.setPower(0);
            //motorExtend.setPower(0);
        }

        if (gamepad1.back)
            motorScale = motorScale == -1 ? -.25 : -1;

        /*if(gamepad2.a) {
            motorWL.setPower(-1);
        } else if (gamepad2.y) {
            motorWL.setPower(1);
        } else {
            motorWL.setPower(0);
        }*/

        //if(gamepad2.b) {
            //servoWinchArm.setPosition(Range.clip(servoWinchArm.getPosition() + .1, 0, 1));
        //    servoWinchArm.setPosition(servoWinchArmUpPos);
        //}
        //else if(gamepad2.a) {
            //servoWinchArm.setPosition(Range.clip(servoWinchArm.getPosition() - .1, 0, 1));
        //    servoWinchArm.setPosition(servoWinchArmDownPos);
        //} else if (gamepad2.y) {
        //    servoWinchArm.setPosition(.8);
        //}


        if (gamepad2.right_bumper) {
            servoLeftManip.setPower(1);
            servoRightManip.setPower(-1);
        } else if (gamepad2.left_bumper) {
            servoLeftManip.setPower(-1);
            servoRightManip.setPower(1);
        } else {
            servoLeftManip.setPower(0);
            servoRightManip.setPower(0);
        }

        if (Math.abs(gamepad2.left_stick_y) > .25) {
            motorExtend.setPower(gamepad2.left_stick_y);
        } else {
            motorExtend.setPower(0);
        }

        if (Math.abs(gamepad2.right_stick_y) > .1) {
            motorManip.setPower(-gamepad2.right_stick_y / 2.0);
        } else {
            motorManip.setPower(0);
        }

        if(gamepad2.a) {
            servoBasket.setPosition(.15);
        } else if(gamepad2.b) {
            servoBasket.setPosition(0);
        }

        if(gamepad2.x) {
            servoGate.setPosition(.25);
        } else if(gamepad2.y) {
            servoGate.setPosition(.5);
        }

        telemetry.addData("Right motor speeds", yR);
        telemetry.addData("Left motor speed", yL);
        telemetry.addData("motorScale: ", motorScale);
        telemetry.addData("Servo Winch Arm Pos", servoWinchArm.getPosition());
        //telemetry.addData("motorFL: ", motorFL.getCurrentPosition());
        telemetry.addData("motorBL: ", motorBL.getCurrentPosition());
        telemetry.addData("motorLiftDown1: ", motorLiftDown1.getCurrentPosition());
        telemetry.addData("motorLiftDown2: ", motorLiftDown2.getCurrentPosition());
        //telemetry.addData("motorFR: ", motorFR.getCurrentPosition());
        //telemetry.addData("motorBR: ", motorBR.getCurrentPosition());
        telemetry.addData("yaw: ", imu.getYaw());
        telemetry.addData("pitch: ", imu.getPitch());
        telemetry.addData("servoBasket: ", servoBasket.getPosition());

        //telemetry.addData("angle error from 45: ", imu.getTrueDiff(45));
        //telemetry.addData("rangeB: ", getDistB());
        //telemetry.addData("rangeL: ", getDistL());
        //telemetry.addData("totalDist: ", getDistB() - getDistL());
        telemetry.update();
    }
}
