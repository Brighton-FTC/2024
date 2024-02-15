package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Code to open/close grabber, and tilt grabber. <br />
 */

public class GrabberComponent {
    // TODO: fill in these values
    public static final int GRABBER_CLOSED_POSITION = 0;
    public static final int GRABBER_OPEN_POSITION = 90;

    private final ServoEx grabberServo1;
    private final ServoEx grabberServo2;

    private boolean isGrabberClosed = true;


    /**
     * Code to open/close grabber, and tilt grabber. <br />
     * @param grabberServo1 One of the servos that controls the grabber.
     * @param grabberServo2 One of the servos that controls the grabber.
     */
    public GrabberComponent(ServoEx grabberServo1, ServoEx grabberServo2) {
        this.grabberServo1 = grabberServo1;
        this.grabberServo1.setRange(GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);

        this.grabberServo2 = grabberServo2;
        this.grabberServo2.setRange(GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);
        this.grabberServo2.setInverted(true);
    }

    /**
     * Open the grabber.
     */
    public void open() {
        grabberServo1.turnToAngle(GRABBER_OPEN_POSITION);
        grabberServo2.turnToAngle(GRABBER_OPEN_POSITION);
        isGrabberClosed = false;
    }

    /**
     * Close the grabber.
     */
    public void close() {
        grabberServo1.turnToAngle(GRABBER_CLOSED_POSITION);
        grabberServo2.turnToAngle(GRABBER_CLOSED_POSITION);
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

    /**
     * Get whether the grabber is closed.
     * @return True if the grabber is closed, false if it is open.
     */
    public boolean isClosed() {
        return isGrabberClosed;
    }

    /**
     * Get the servos.
     * @return An array of two {@link ServoEx} objects that the grabber uses.
     */
    public ServoEx[] getServos() {
        return new ServoEx[]{grabberServo1, grabberServo2};
    }
}