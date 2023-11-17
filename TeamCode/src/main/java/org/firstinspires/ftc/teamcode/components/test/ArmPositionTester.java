package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get the motor encoder positions for the arm. <br />
 *
 * Controls:
 * <ul>
 *     <li>Rotate servo by rotation angle - dpad left & dpad right</li>
 *     <li>Change rotation angle  - dpad down & dpad up</li>
 *     <li>Rotate arm  - left joystick</li>
 * </ul>
 *
 */

@TeleOp(name = "Grabber Arm Position Tester", group = "components-test")
public class ArmPositionTester extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private Motor armMotor;

    private ServoEx grabberTiltServo;

    private int rotationAngle = 20;

    @Override
    public void init() {
        grabberTiltServo = new SimpleServo(hardwareMap, "servo_name", 0, 360);
        armMotor = new Motor(hardwareMap, "motorOne");
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
            grabberTiltServo.rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            grabberTiltServo.rotateByAngle(-rotationAngle);
        }

        armMotor.set(gamepad.getLeftX());
        armMotor.setRunMode(Motor.RunMode.RawPower);

        telemetry.addData("Servo's current angle", grabberTiltServo.getAngle());
        telemetry.addData("Motor position", armMotor.getCurrentPosition());
    }
}
