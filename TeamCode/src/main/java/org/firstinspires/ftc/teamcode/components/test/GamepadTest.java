package org.firstinspires.ftc.teamcode.components.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gamepad Test")
public class GamepadTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (true) {
            gamepad1.rumble(100);
        }
    }
}
