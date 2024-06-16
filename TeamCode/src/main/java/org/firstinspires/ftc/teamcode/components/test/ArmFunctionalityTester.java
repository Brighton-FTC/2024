package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Code to test the functionality of the arm. <br />
 * This is an example for how to use the {@link ArmComponent} class. <br />
 * <p>
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
                new MotorEx(hardwareMap, "arm_motor")
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        armComponent.read();
        armComponent.moveToSetPoint();

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (armComponent.getState() == ArmComponent.State.PLACE_HIGH_BACKDROP) {
                armComponent.setState(ArmComponent.State.PLACE_LOW_BACKDROP);
            } else {
                armComponent.setState(ArmComponent.State.PLACE_HIGH_BACKDROP);
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            armComponent.setState(ArmComponent.State.PICKUP_GROUND);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
            armComponent.getArmMotor().resetEncoder();
        }

        if (gamepad.getLeftX() > 0.2) {
            armComponent.getArmMotor().set(gamepad.getLeftX());
        }

        telemetry.addData("Arm state:", armComponent.getState());
        telemetry.addData("Setpoint: ", armComponent.getSetPoint());
        telemetry.addData("Motor position", armComponent.getArmMotor().getCurrentPosition());
    }
}
