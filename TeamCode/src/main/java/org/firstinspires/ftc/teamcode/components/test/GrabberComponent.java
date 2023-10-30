package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Code to open/close grabber, and tilt grabber. <br />
 * Controls:
 * <ul>
 *     <li>Open/close grabber: A (Cross on PlayStation controller)</li>
 *     <li>Tilt grabber up/down: B (Circle on PlayStation controller)</li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Grabber component (untested)", group = "components_test")
public class GrabberComponent extends OpMode {
    // TODO: fill in these values
    public final int GRABBER_CLOSED_POSITION = 0;
    public final int GRABBER_OPEN_POSITION = 90;
    public final int GRABBER_TILT_DOWN_POSITION = 0;
    public final int GRABBER_TILT_UP_POSITION = 180;

    private ServoEx grabberServo;
    private ServoEx grabberTiltServo;

    private boolean isGrabberClosed = false;
    private boolean isGrabberTiltedUp = false;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void init() {
        // TODO: fill in device names
        grabberServo = (ServoEx) hardwareMap.servo.get("device name here");
        grabberTiltServo = (ServoEx) hardwareMap.servo.get("device name here as well");

        // set ranges on servos, just in case
        grabberServo.setRange(GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);
        grabberTiltServo.setRange(GRABBER_TILT_DOWN_POSITION, GRABBER_TILT_UP_POSITION);
    }

    // I don't know if this is necessary, but I put it in here for now
    @Override
    public void start() {
        // set servos to their starting positions
        grabberServo.setPosition(GRABBER_OPEN_POSITION);
        grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(Button.A)) {
            if (isGrabberClosed) {
                grabberServo.setPosition(GRABBER_OPEN_POSITION);
            } else {
                grabberServo.setPosition(GRABBER_OPEN_POSITION);
            }

            isGrabberClosed = !isGrabberClosed;
        }

        if (gamepad.wasJustPressed(Button.B)) {
            if (isGrabberTiltedUp) {
                grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
            } else {
                grabberTiltServo.setPosition(GRABBER_TILT_UP_POSITION);
            }

            isGrabberTiltedUp = !isGrabberTiltedUp;
        }

        telemetry.addData("Grabber Position", isGrabberClosed ? "Closed" : "Open");
        telemetry.addData("Grabber Tilt Angle", grabberTiltServo.getAngle());
    }
}