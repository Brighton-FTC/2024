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

@TeleOp(name = "Grabber Servo Position Finder", group = "grabber-test")
public class GrabberPositionFinder extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private ServoEx testServo;

    private int rotationAngle = 20;

    @Override
    public void init() {
        testServo = new SimpleServo(hardwareMap, "servo_name", 0, 360);
    }

    @Override
    public void loop() {
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
