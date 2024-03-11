package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "Active Intake Functionality Tester", group = "active-intake-test")
public class ActiveIntakeFunctionalityTester extends OpMode {
    private ActiveIntakeComponent activeIntake;
    private GamepadEx gamepad;

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);

        activeIntake = new ActiveIntakeComponent(
                new MotorEx(hardwareMap, "active_intake_motor_left"),
                new MotorEx(hardwareMap, "active_intake_motor_right")
        );
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            activeIntake.turnManually();
        }

        if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            activeIntake.turnContinually();
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            activeIntake.turnMotorOff();
        }

        activeIntake.moveMotor();

        telemetry.addData("State:", activeIntake.getState().toString());
    }
}