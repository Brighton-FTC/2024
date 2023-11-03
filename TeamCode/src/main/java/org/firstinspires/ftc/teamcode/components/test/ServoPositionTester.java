package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the servo positions for the grabber. <br />
 *
 * Controls:
 * <ul>
 *     <li>Change servo - dpad left & dpad right</li>
 *     <li>Change rotation angle - dpad down & dpad up</li>
 *     <li>Rotate servo by rotation angle - A (cross on playstation) and B (circle on playstation)</li>
 * </ul>
 *
 */

@TeleOp(name = "Grabber Servo Position Tester", group = "components_test")
public class ServoPositionTester extends OpMode {
    private ServoEx[] servos;
    private int currentServoIndex = 0;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private int rotationAngle = 20;

    @Override
    public void init() {
        servos = new ServoEx[]{
                new SimpleServo(hardwareMap, "servo_name", 0, 360),
                new SimpleServo(hardwareMap, "servo_name", 0, 360)
        };
    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            currentServoIndex--;
            currentServoIndex %= servos.length;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            currentServoIndex++;
            currentServoIndex %= servos.length;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            rotationAngle -= 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            rotationAngle += 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            servos[currentServoIndex].rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
            servos[currentServoIndex].rotateByAngle(-rotationAngle);
        }

        telemetry.addData("Current servo index", currentServoIndex);
        telemetry.addData("Rotation angle", rotationAngle);
        telemetry.addData("Current servo angle", servos[currentServoIndex].getAngle());
    }
}
