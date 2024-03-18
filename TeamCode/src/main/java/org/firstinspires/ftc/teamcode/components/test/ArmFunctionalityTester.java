package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

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

    private ArmComponent.State selectedState;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // this is how you get the voltage - just takes first hub
        // not sure if you should be getting average of all voltages but prob doesn't matter
        armComponent = new ArmComponent(
                new MotorEx(hardwareMap, "arm_motor"),
                new SimpleServo(hardwareMap, "outtake_rotation_servo", 0, 360),
                allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS)
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        armComponent.read();
        armComponent.moveToSetPoint();

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if (selectedState == ArmComponent.State.PLACE_GROUND) {
                selectedState = ArmComponent.State.LOW;
            } else if (selectedState == ArmComponent.State.LOW) {
                selectedState = ArmComponent.State.MIDDLE;
            } else if (selectedState == ArmComponent.State.MIDDLE) {
                selectedState = ArmComponent.State.HIGH;
            } else {
                selectedState = ArmComponent.State.PLACE_GROUND;
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (selectedState == ArmComponent.State.PLACE_GROUND) {
                selectedState = ArmComponent.State.HIGH;
            } else if (selectedState == ArmComponent.State.HIGH) {
                selectedState = ArmComponent.State.MIDDLE;
            } else if (selectedState == ArmComponent.State.MIDDLE){
                selectedState = ArmComponent.State.LOW;
            } else {
                selectedState = ArmComponent.State.PLACE_GROUND;
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            armComponent.setState(selectedState);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            armComponent.setState(ArmComponent.State.PICKUP_GROUND);
        }

        telemetry.addData("Arm state:", armComponent.getState());
        telemetry.addData("Selected state:", selectedState.toString());
        telemetry.addData("Setpoint: ", armComponent.getSetPoint());
        telemetry.addData("Servo position", armComponent.getOuttakeRotationServo().getAngle());
        telemetry.update();
    }
}
