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
    private ServoEx rotationServo;

    @Override
    public void init() {
        armMotor = new Motor(hardwareMap, "arm_motor");
        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        rotationServo = new SimpleServo(hardwareMap, "outtake_rotation_servo", 0, 360);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            rotationServo.rotateByAngle(-15);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            rotationServo.rotateByAngle(15);
        }

        armMotor.set(gamepad.getLeftX() / 2);

        telemetry.addData("Motor position", armMotor.getCurrentPosition());
        telemetry.addData("Servo pos", rotationServo.getPosition());
    }
}
