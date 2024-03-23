package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Code to open/close outtake, and tilt outtake. <br />
 */

public class OuttakeComponent {
    // TODO: fill in these values
    public static final double RELEASE_ANGLE = 40;
    public static final double RELEASE_ALL_ANGLE = 60;

    public static long SERVO_SLEEP_TIME = 200;

    private final ServoEx outtakeServo;

    private boolean isOuttakeClosed = true;


    /**
     * Code to open/close outtake, and tilt outtake. <br />
     * @param outtakeServo The servo that controls the outtake.
     */
    public OuttakeComponent(ServoEx outtakeServo) {
        this.outtakeServo = outtakeServo;
        this.outtakeServo.setRange(0, 360);
    }

    public void release(double angle) {
        outtakeServo.rotateByAngle(angle);
        isOuttakeClosed = false;
        try {
            Thread.sleep(SERVO_SLEEP_TIME);
        } catch (InterruptedException e) {

        }
        outtakeServo.rotateByAngle(-angle);
        isOuttakeClosed = true;
    }

    public void releasePixel() {
        release(RELEASE_ANGLE);
    }

    public void releaseAllPixels() {
        release(RELEASE_ALL_ANGLE);
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