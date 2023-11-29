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

    private ServoEx testServo1;
    private ServoEx testServo2;

    private int rotationAngle = 20;

    @Override
    public void init() {
        testServo1 = new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360);

        testServo2 = new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360);
        testServo2.setInverted(true);
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
            testServo1.rotateByAngle(rotationAngle);
            testServo2.rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            testServo1.rotateByAngle(-rotationAngle);
            testServo2.rotateByAngle(-rotationAngle);
        }

        telemetry.addData("Rotation angle", rotationAngle);
    }
}
