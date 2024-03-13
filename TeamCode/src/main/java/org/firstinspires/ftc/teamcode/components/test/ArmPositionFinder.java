package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the motor encoder and servo positions for the arm. <br />
 *
 * Controls:
 * <ul>
 *     <li>Change rotation angle  - dpad down & dpad up</li>
 *     <li>Rotate arm  - left joystick</li>
 * </ul>
 *
 */

@TeleOp(name = "Arm Position Tester", group = "arm-test")
public class ArmPositionFinder extends OpMode {
    private GamepadEx gamepad;

    private Motor armMotor;

    @Override
    public void init() {
        armMotor = new Motor(hardwareMap, "arm_motor");
        armMotor.setRunMode(Motor.RunMode.RawPower);
        gamepad = new GamepadEx(gamepad1);

        System.out.println(gamepad1 == null);
    }

    @Override
    public void loop() {
        armMotor.set(gamepad.getLeftX());

        telemetry.addData("Motor position", armMotor.getCurrentPosition());
    }
}
