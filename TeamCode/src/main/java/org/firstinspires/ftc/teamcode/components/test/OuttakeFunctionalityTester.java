package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;


/**
 * OpMode to test the functionality of the grabber.
 */
@TeleOp(name = "Outtake Functionality Tester", group = "outtake-test")
public class OuttakeFunctionalityTester extends OpMode {
    private OuttakeComponent outtakeComponent;

    private GamepadEx gamepad;

    @Override
    public void init() {
        outtakeComponent = new OuttakeComponent(
                new SimpleServo(hardwareMap, "outtake_servo_front",
                        0, 360),
                new SimpleServo(hardwareMap, "outtake_servo_back",
                        0, 360)
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            outtakeComponent.releaseBack();
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            outtakeComponent.releaseFront();
        }

        if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            outtakeComponent.releaseAll();
        }

        telemetry.addLine(outtakeComponent.areServosTurned().first? "Front Servo Turned" : "Front Servo not Turned");
        telemetry.addLine(outtakeComponent.areServosTurned().second ? "Back Servo Turned" : "Back Servo not Turned");
    }
}
