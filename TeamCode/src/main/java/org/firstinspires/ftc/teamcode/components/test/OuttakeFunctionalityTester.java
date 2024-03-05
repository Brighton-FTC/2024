package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * OpMode to test the functionality of the grabber.
 */
@TeleOp(name = "Outtake Functionality Tester", group = "outtake-test")
public class OuttakeFunctionalityTester extends OpMode {
    OuttakeComponent outtakeComponent;

    GamepadEx gamepad;

    @Override
    public void init() {
        outtakeComponent = new OuttakeComponent(
                new SimpleServo(hardwareMap, "outtake_servo",
                        0, 360)
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            outtakeComponent.releasePixel();
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            outtakeComponent.releaseAllPixels();
        }

        telemetry.addLine(outtakeComponent.isClosed() ? "Outtake Closed" : "Outtake Open");
    }
}
