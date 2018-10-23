package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Distance Test", group="TrollBot")

public class DistanceTest extends CustomOpMode {
    public void init() {
        initizialize();
    }
    public void loop() {
        telemetry.addData("Front Distance", getDist());
    }
}
