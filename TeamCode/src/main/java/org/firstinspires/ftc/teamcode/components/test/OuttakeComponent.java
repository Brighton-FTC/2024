package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Code to open/close outtake, and tilt outtake. <br />
 */

public class OuttakeComponent {
    // TODO: fill in these values
    public static final int OUTTAKE_CLOSED_POSITION = 0;
    public static final int OUTTAKE_OPEN_POSITION = 90;
    public static final long RELEASE_TIME = 500;

    private final ServoEx outtakeServo;

    private boolean isOuttakeClosed = true;


    /**
     * Code to open/close outtake, and tilt outtake. <br />
     * @param outtakeServo The servo that controls the outtake.
     */
    public OuttakeComponent(ServoEx outtakeServo) {
        this.outtakeServo = outtakeServo;
        this.outtakeServo.setRange(OUTTAKE_CLOSED_POSITION, OUTTAKE_OPEN_POSITION);
    }

    /**
     * Open the outtake.
     */
    public void open() {
        outtakeServo.turnToAngle(OUTTAKE_OPEN_POSITION);
        isOuttakeClosed = false;
    }

    /**
     * Close the outtake.
     */
    public void close() {
        outtakeServo.turnToAngle(OUTTAKE_CLOSED_POSITION);
        isOuttakeClosed = true;
    }

    public void releasePixel() {
        open();
        try {
            Thread.sleep(RELEASE_TIME);
        } catch (InterruptedException ignored) {}
        close();
    }

    /**
     * Get whether the outtake is closed.
     * @return True if the outtake is closed, false if it is open.
     */
    public boolean isClosed() {
        return isOuttakeClosed;
    }

    /**
     * Get the servo.
     * @return The outtake servo.
     */
    public ServoEx getServo() {
        return outtakeServo;
    }
}