package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * Code to test the functionality of the arm. <br />
 * This is an example for how to use the {@link ArmComponent} class. <br />
 *
 * Controls:
 * <ul>
 *     <li>Cross - toggle arm position. </li>
 * </ul>
 */
@TeleOp(name = "Arm Functionality Tester", group = "arm-test")
public class ArmFunctionalityTester extends OpMode {
    private ArmComponent armComponent;
    private GamepadEx gamepad;

    @Override
    public void init() {
        armComponent = new ArmComponent(
                new MotorEx(hardwareMap, "arm_motor"),
                new SimpleServo(hardwareMap, "grabber_rotator_servo", ArmComponent.GRABBER_ROTATE_DOWN_POSITION, ArmComponent.GRABBER_ROTATE_UP_POSITION)
        );

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(armComponent::toggle);
    }

    @Override
    public void loop() {
        armComponent.moveToSetPoint();
    }
}
