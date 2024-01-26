package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingMotorEx;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

import java.util.List;

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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // this is how you get the voltage - just takes first hub
        // not sure if you should be getting average of all voltages but prob doesn't matter
        armComponent = new ArmComponent(
                new FTCLibCachingMotorEx(hardwareMap, "arm_motor"),
                allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS)
        );

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(armComponent::toggle);
    }

    @Override
    public void loop() {
        armComponent.read();
        armComponent.moveToSetPoint();

        telemetry.addData("Is arm lifted? ", armComponent.isLifted());
        telemetry.addData("Setpoint: ", armComponent.getSetPoint());
        telemetry.update();
    }
}
