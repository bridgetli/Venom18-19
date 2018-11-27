package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by bodeng on 11/17/18.
 */

@Autonomous(name = "AdjustableDelayTest", group = "test")
public class AdjustableDelayTest extends CustomLinearOpMode {
    ElapsedTime time = new ElapsedTime();
    public void runOpMode() {
        int secDelay = 0;
        while (!opModeIsActive()) {
            if (gamepad1.dpad_up) {
                secDelay++;
                sleep(250);
            } else if (gamepad1.dpad_down) {
                secDelay--;
                sleep(250);
            }
            telemetry.addData("Seconds of Delay: ", secDelay);
            telemetry.update();
        }
        waitForStart();
        time.reset();
        while (time.seconds() < secDelay && opModeIsActive()) {
            telemetry.addData("Time left: ", secDelay - (int) time.seconds() + "");
            telemetry.update();
        }
        telemetry.clear();
        telemetry.addLine("AUTO IS RUNNING BOIS");
        telemetry.update();
    }
}
