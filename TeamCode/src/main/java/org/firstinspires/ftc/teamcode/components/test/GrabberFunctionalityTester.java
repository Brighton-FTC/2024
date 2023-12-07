package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * OpMode to test the functionality of the grabber.
 */
@TeleOp(name = "Grabber Functionality Tester", group = "grabber-test")
public class GrabberFunctionalityTester extends OpMode {
    GrabberComponent grabberComponent;

    GamepadEx gamepad;

    @Override
    public void init() {
        grabberComponent = new GrabberComponent(
                new SimpleServo(hardwareMap, "grabber_servo1",
                        GrabberComponent.GRABBER_CLOSED_POSITION,
                        GrabberComponent.GRABBER_OPEN_POSITION),
                new SimpleServo(hardwareMap, "grabber_servo_2",
                        GrabberComponent.GRABBER_CLOSED_POSITION,
                        GrabberComponent.GRABBER_CLOSED_POSITION)
        );

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(grabberComponent::toggle);

        telemetry.addLine(grabberComponent.isClosed() ? "Grabber Closed" : "Grabber Open");
    }

    @Override
    public void loop() {

    }
}
