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
 *     <li>Rotate servo by rotation angle - dpad left & dpad right</li>
 *     <li>Change rotation angle  - dpad down & dpad up</li>
 *     <li>Rotate arm  - left joystick</li>
 * </ul>
 *
 */

@TeleOp(name = "Grabber Arm Position Tester", group = "arm-test")
public class ArmPositionFinder extends OpMode {
    private GamepadEx gamepad;

    private Motor armMotor;

    @Override
    public void init() {
        armMotor = new Motor(hardwareMap, "arm_motor");
        gamepad = new GamepadEx(gamepad1);

        System.out.println(gamepad1 == null);
    }

    @Override
    public void loop() {
        armMotor.set(gamepad.getLeftX());
        armMotor.setRunMode(Motor.RunMode.RawPower);

        telemetry.addData("Motor position", armMotor.getCurrentPosition());
    }
}
