package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the servo positions for the grabber. <br />
 * <p>
 * Controls:
 * <ul>
 *     <li>Rotate servo - dpad left & dpad right</li>
 *     <li>Change rotation angle - dpad down & dpad up</li>
 * </ul>
 */

@TeleOp(name = "Outtake Servo Position Finder", group = "outtake-test")
public class OuttakePositionFinder extends OpMode {
    private GamepadEx gamepad;

    private ServoEx testServo;

    private int rotationAngle = 20;

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);
        testServo = new SimpleServo(hardwareMap, "outtake_servo", 0, 360);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            rotationAngle -= 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            rotationAngle += 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            testServo.rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            testServo.rotateByAngle(-rotationAngle);
        }

        telemetry.addData("Rotation angle", rotationAngle);
    }
}
