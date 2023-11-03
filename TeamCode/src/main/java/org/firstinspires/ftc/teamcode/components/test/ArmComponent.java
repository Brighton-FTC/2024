package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Code to lift/lower arm. <br />
 * Controls:
 * <ul>
 *     <li>Lift/Lower Arm: B (Circle on PlayStation controller)</li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Arm Component", group = "components_test")
public class ArmComponent extends OpMode {
    // TODO: fill in these values
    public final int GRABBER_TILT_DOWN_POSITION = 0;
    public final int GRABBER_TILT_UP_POSITION = 180;
    public final int ARM_LIFTED_POSITION = 0;
    public final int ARM_NONLIFTED_POSITION = 2000;

    public static final GamepadKeys.Button SET_ARM_LIFTED = GamepadKeys.Button.B;

    private ServoEx grabberTiltServo;

//    private boolean isGrabberTiltedUp = false;
    private boolean isArmLifted = false;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void init() {
        // TODO: fill in device names
        grabberTiltServo = (ServoEx) hardwareMap.servo.get("device name here as well");

        // set ranges on servos, just in case
        grabberTiltServo.setRange(GRABBER_TILT_DOWN_POSITION, GRABBER_TILT_UP_POSITION);

        gamepad.getGamepadButton(SET_ARM_LIFTED).whenPressed(new InstantCommand(() -> {
            isArmLifted = !isArmLifted;
        }));
    }

    // I don't know if this is necessary, but I put it in here for now
    @Override
    public void start() {
        // set servos to their starting positions
        grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
    }

    @Override
    public void loop() {
        if (isArmLifted) {
            grabberTiltServo.setPosition(GRABBER_TILT_UP_POSITION);
        } else {
            grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
        }

        telemetry.addData("Grabber Tilt Angle", grabberTiltServo.getAngle());
    }
}