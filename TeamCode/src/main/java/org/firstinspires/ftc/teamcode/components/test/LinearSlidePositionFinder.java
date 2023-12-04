package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the motor encoder positions for the arm. <br />
 * <p>
 * Controls:
 * <ul>
 *     <li>Rotate arm  - left joystick</li>
 * </ul>
 */

@TeleOp(name = "Linear Slide Position Finder", group = "linear-slide-test")
public class LinearSlidePositionFinder extends OpMode {
    private GamepadEx gamepad;

    private MotorEx linearSlideMotor;

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_motor");
    }

    @Override
    public void loop() {
        linearSlideMotor.set(gamepad.getLeftX());
        linearSlideMotor.setRunMode(Motor.RunMode.RawPower);

        telemetry.addData("Motor position", linearSlideMotor.getCurrentPosition());
    }
}