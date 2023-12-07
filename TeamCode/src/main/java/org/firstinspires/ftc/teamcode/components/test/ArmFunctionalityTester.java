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

    private boolean previousCrossValue = false; // store for toggle

    @Override
    public void init() {
        armComponent = new ArmComponent(
                new MotorEx(hardwareMap, "arm_motor")
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepad.getButton(PSButtons.CROSS) && !previousCrossValue) {
            armComponent.toggle();
        }

        previousCrossValue = gamepad.getButton(PSButtons.CROSS);

        armComponent.moveToSetPoint();

        telemetry.addData("Is arm lifted? ", armComponent.isLifted());
        telemetry.addData("Setpoint: ", armComponent.getSetPoint());
        telemetry.update();
    }
}
