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

//    private ServoEx grabberRotatorServo;

    private int rotationAngle = 20;

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);

//        grabberRotatorServo = new SimpleServo(hardwareMap, "grabber_rotator_servo", 0, 360);
        armMotor = new Motor(hardwareMap, "arm_motor");
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
//            grabberRotatorServo.rotateByAngle(rotationAngle);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//            grabberRotatorServo.rotateByAngle(-rotationAngle);
        }

        armMotor.set(gamepad.getLeftX());
        armMotor.setRunMode(Motor.RunMode.RawPower);

//        telemetry.addData("Servo's current angle", grabberRotatorServo.getAngle());
        telemetry.addData("Motor position", armMotor.getCurrentPosition());
    }
}
