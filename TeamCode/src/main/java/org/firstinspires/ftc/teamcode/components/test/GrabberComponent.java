package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Code to open/close grabber, and tilt grabber. <br />
 * Controls:
 * <ul>
 *     <li>Open/close grabber: Right bumper</li>
 *     <li>Tilt grabber up: Right lever</li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Grabber component (untested)", group = "components_test")
public class GrabberComponent extends OpMode {
    // TODO: fill in these values
    public final int GRABBER_CLOSED_POSITION = 0;
    public final int GRABBER_OPEN_POSITION = 90;
    public final int MIN_GRABBER_TILT_POSITION = 0;
    public final int MAX_GRABBER_TILT_POSITION = 90;

    private ServoEx grabberServo;
    private ServoEx grabberTiltServo;

    private boolean isGrabberClosed = true;
    // private boolean isGrabberTiltedDown = true;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void init() {
        // TODO: fill in device names
        grabberServo = new SimpleServo(hardwareMap, "servo_name", GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);
        grabberTiltServo = new SimpleServo(hardwareMap, "servo_name", MIN_GRABBER_TILT_POSITION, MAX_GRABBER_TILT_POSITION);
    }

    @Override
    public void start() {
        // set servos to their starting positions
        grabberServo.setPosition(GRABBER_OPEN_POSITION);
        grabberTiltServo.setPosition(MIN_GRABBER_TILT_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(Button.RIGHT_BUMPER)) {
            if (isGrabberClosed) {
                grabberServo.setPosition(GRABBER_OPEN_POSITION);
            } else {
                grabberServo.setPosition(GRABBER_CLOSED_POSITION);
            }

            isGrabberClosed = !isGrabberClosed;
        }

        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            grabberTiltServo.setPosition(MAX_GRABBER_TILT_POSITION);
        } else {
            grabberTiltServo.setPosition(MIN_GRABBER_TILT_POSITION);
        }


        telemetry.addData("Grabber Position", isGrabberClosed ? "Closed" : "Open");
        telemetry.addData("Grabber Tilt Angle", grabberTiltServo.getAngle());
    }
}