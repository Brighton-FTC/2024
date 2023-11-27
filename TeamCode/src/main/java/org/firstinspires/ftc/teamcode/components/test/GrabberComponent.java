package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Code to open/close grabber, and tilt grabber. <br />
 */

public class GrabberComponent {
    // TODO: fill in these values
    public static final int GRABBER_CLOSED_POSITION = 0;
    public static final int GRABBER_OPEN_POSITION = 90;

    private final ServoEx grabberServo;

    private boolean isGrabberClosed = true;


    /**
     * Code to open/close grabber, and tilt grabber. <br />
     * @param grabberServo The servo that controls the grabber.
     */
    public GrabberComponent(ServoEx grabberServo) {
        this.grabberServo = grabberServo;
        this.grabberServo.setRange(GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);
    }

    /**
     * Open the grabber.
     */
    public void open() {
        grabberServo.turnToAngle(GRABBER_OPEN_POSITION);
        isGrabberClosed = false;
    }

    /**
     * Close the grabber.
     */
    public void close() {
        grabberServo.turnToAngle(GRABBER_CLOSED_POSITION);
        isGrabberClosed = true;
    }

    /**
     * If the grabber is open, then close it, otherwise open it.
     */
    public void toggle() {
        if (isGrabberClosed) {
            open();
        } else {
            close();
        }

        isGrabberClosed = !isGrabberClosed;
    }

    public boolean isClosed() {
        return isGrabberClosed;
    }
}