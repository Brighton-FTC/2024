package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get servo for drone launcher.
 * Controls:
 * <ul>
 *     <li>Rotate servo - dpad left & dpad right</li>
 *     <li>Change rotation angle - dpad down & dpad up</li>
 * </ul>
 */

@TeleOp(name = "Drone Launcher Position Tester", group = "drone-test")
public class DroneLauncherPositionFinder extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private ServoEx droneLauncherServo;

    private int rotationAngle = 20;

    @Override
    public void init() {
        droneLauncherServo = new SimpleServo(hardwareMap, "drone_servo", 0, 360);
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
            droneLauncherServo.rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            droneLauncherServo.rotateByAngle(-rotationAngle);
        }

        telemetry.addData("Rotation angle", rotationAngle);
        telemetry.addData("Motor position", droneLauncherServo.getAngle());
    }
}