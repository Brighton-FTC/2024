package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

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
                new SimpleServo(hardwareMap, "drone_servo",
                        DroneLauncherComponent.READY_POSITION,
                        DroneLauncherComponent.LAUNCH_POSITION)
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
       gamepad.readButtons();

       if (gamepad.wasJustPressed(PSButtons.CROSS)) {
           droneLauncherComponent.launch();
       }
    }
}