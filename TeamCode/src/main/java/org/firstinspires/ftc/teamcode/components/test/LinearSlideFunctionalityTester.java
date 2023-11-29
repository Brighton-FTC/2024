package org.firstinspires.ftc.teamcode.components.test;

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

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(linearSlide::toggle);
    }

    @Override
    public void loop() {
        linearSlide.moveToSetPoint();

        telemetry.addLine(linearSlide.isLowered() ? "Linear slide is lowered" : "Linear slide is lifted.");
    }
}
