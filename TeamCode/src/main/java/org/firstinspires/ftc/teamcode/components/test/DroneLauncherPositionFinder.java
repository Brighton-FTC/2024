package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the motor encoder positions for drone launcher.
 * Use the joystick for large adjustments, and dpad left & right for small adjustments. <br />
 *
 * Controls:
 * <ul>
 *     <li>Rotate motor by small increments - dpad left & dpad right</li>
 *     <li>Change how much motor rotates  - dpad down & dpad up</li>
 *     <li>Rotate motor by large increments  - left joystick left & right</li>
 * </ul>
 *
 */

@TeleOp(name = "Drone Launcher Position Tester", group = "drone-test")
public class DroneLauncherPositionFinder extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private Motor droneLauncherMotor;

    private int motorIncrement = 20;

    private int currentTargetPosition = 0;

    private final int gamepadMultiplier = 10;

    @Override
    public void init() {
        droneLauncherMotor = new Motor(hardwareMap, "motorOne");
        droneLauncherMotor.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            motorIncrement -= 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            motorIncrement += 5;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            currentTargetPosition += motorIncrement;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            currentTargetPosition += motorIncrement;
        }

        currentTargetPosition += gamepad.getLeftX() * motorIncrement * gamepadMultiplier;

        droneLauncherMotor.setTargetPosition(currentTargetPosition);

        telemetry.addData("Motor increment", motorIncrement);
        telemetry.addData("Motor position", droneLauncherMotor.getCurrentPosition());
    }
}