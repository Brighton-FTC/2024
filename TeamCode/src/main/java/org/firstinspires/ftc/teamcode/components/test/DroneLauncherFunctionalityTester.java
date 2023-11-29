package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * Code to test the functionality of the drone launcher. <br />
 * This is an example for how to use the {@link DroneLauncherComponent} class. <br />
 *
 * Controls:
 * <ul>
 *     <li>Cross - launch drone. </li>
 * </ul>
 */
@TeleOp(name = "Drone Launcher Functionality Tester", group = "drone-test")
public class DroneLauncherFunctionalityTester extends OpMode {
    private DroneLauncherComponent droneLauncherComponent;
    private GamepadEx gamepad;

    @Override
    public void init() {
        droneLauncherComponent = new DroneLauncherComponent(
                new MotorEx(hardwareMap, "drone_motor")
        );

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(droneLauncherComponent::launch);
    }

    @Override
    public void loop() {
    }
}