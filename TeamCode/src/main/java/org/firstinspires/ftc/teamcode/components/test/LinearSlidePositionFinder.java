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

@TeleOp(name = "Grabber Arm Position Tester", group = "components-test")
public class LinearSlidePositionFinder extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private MotorEx linearSlideMotor;

    @Override
    public void init() {
        linearSlideMotor = new MotorEx(hardwareMap, "motorOne");
    }

    @Override
    public void loop() {
        linearSlideMotor.set(gamepad.getLeftX());
        linearSlideMotor.setRunMode(Motor.RunMode.RawPower);

        telemetry.addData("Motor position", linearSlideMotor.getCurrentPosition());
    }
}