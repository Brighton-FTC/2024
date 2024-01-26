package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingMotorEx;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

import java.util.List;

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

    private ArmComponent arm;

    private GamepadEx gamepad;

    @Override
    public void init() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        arm = new ArmComponent(new FTCLibCachingMotorEx(hardwareMap, "arm_motor"),
                allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS));
        linearSlide = new LinearSlideComponent(new FTCLibCachingMotorEx(hardwareMap, "linear_slide_motor"),
                arm,
                allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS));

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.SQUARE).whenPressed(linearSlide::toggle);
    }
    
    @Override
    public void loop() {
        linearSlide.read();
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
        gamepad.readButtons();
        arm.read();
    }
}
