package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Code to open/close outtake, and tilt outtake. <br />
 */

public class OuttakeComponent {
    // TODO: fill in these values
    public static double RELEASE_ANGLE = 23;
    public static double RELEASE_ALL_ANGLE = 40;

    public static long SERVO_SLEEP_TIME = 200;
    private final ElapsedTime elapsedTime = new ElapsedTime();

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
        elapsedTime.reset();
        outtakeServo.rotateByAngle(angle);
        isOuttakeClosed = false;
        while (elapsedTime.milliseconds() < SERVO_SLEEP_TIME) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {}
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

    public Action releaseAction(double turnAngle) {
        return new Action() {
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    outtakeServo.turnToAngle(turnAngle);
                    elapsedTime.reset();
                    init = true;
                }

                return elapsedTime.milliseconds() < SERVO_SLEEP_TIME;
            }
        };
    }

    public Action releasePixelAction() {
        return releaseAction(RELEASE_ANGLE);
    }

    public Action releaseAllPixelsAction() {
        return releaseAction(RELEASE_ALL_ANGLE);
    }
}