package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * Code to test the functionality of the {@link LinearSlideComponent class}. <br />
 * Controls:
 * <ul>
 *     <li>Cross - lift/lower linear slide. </li>
 * </ul>
 */
@TeleOp(name = "Linear slide functionality tester", group = "linear-slide-test")
public class LinearSlideFunctionalityTester extends OpMode {
    private LinearSlideComponent linearSlide;

    private GamepadEx gamepad;

    @Override
    public void init() {
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linear_slide_motor"));

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.SQUARE).whenPressed(linearSlide::toggle);
    }


    @Override
    public void loop() {
        linearSlide.moveToSetPoint();

        telemetry.addLine(linearSlide.isLifted() ? "Linear slide is lifted" : "Linear slide is lowered");
        telemetry.addData("setpoint: ",linearSlide.getSetPoint());


        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            linearSlide.lift();
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            linearSlide.lower();
        }

        CommandScheduler.getInstance().run();
    }
}
