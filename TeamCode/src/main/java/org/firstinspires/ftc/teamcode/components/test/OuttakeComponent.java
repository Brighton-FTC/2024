package org.firstinspires.ftc.teamcode.components.test;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Code to open/close outtake, and tilt outtake. <br />
 */

@Config
public class OuttakeComponent {
    // TODO: fill in these values, and then make them final.
    public static double RELEASE_ANGLE = 90;
    public static long RELEASE_WAIT_TIME = 500;

    private final ServoEx frontServo, backServo;

    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);


    /**
     * Code to open/close outtake, and tilt outtake. <br />
     *
     * @param frontServo The servo at the front of the outtake (nearest the intake).
     * @param backServo  The servo at the back of the outtake (nearest the arm).
     */
    public OuttakeComponent(ServoEx frontServo, ServoEx backServo) {
        this.frontServo = frontServo;
        this.backServo = backServo;
    }

    /**
     * Release all pixels.
     */
    public void releaseAll() {
        releaseBack();
        releaseFront();
    }

    /**
     * Release the back pixel.
     */
    public void releaseBack() {
        backServo.rotateByAngle(RELEASE_ANGLE);

        scheduler.schedule(() -> {
            backServo.rotateByAngle(-RELEASE_ANGLE);
        }, RELEASE_WAIT_TIME, TimeUnit.MILLISECONDS);
    }

    /**
     * Release the front pixel (won't work unless back pixel has already been released).
     */
    public void releaseFront() {
        frontServo.rotateByAngle(RELEASE_ANGLE);

        scheduler.schedule(() -> {
            frontServo.rotateByAngle(-RELEASE_ANGLE);
        }, RELEASE_WAIT_TIME, TimeUnit.MILLISECONDS);
    }

    /**
     * Release all pixels (returns a {@link Action}).
     */
    public Action releaseAllAction() {
        return (telemetryPacket) -> {
            releaseAll();
            return false;
        };
    }

    /**
     * Release the front pixel (returns a {@link Action}).
     */
    public Action releaseFrontAction() {
        return (telemetryPacket) -> {
            releaseFront();
            return false;
        };
    }

    /**
     * Release the front pixel (returns a {@link Action}).
     */
    public Action releaseBackAction() {
        return (telemetryPacket) -> {
            releaseBack();
            return false;
        };
    }

    public ServoEx getFrontServo() {
        return frontServo;
    }

    public ServoEx getBackServo() {
        return backServo;
    }

    /**
     * Get whether the servos are turned.
     * @return A {@link Pair>} object, in the form [is front servo turned, is back servo turned].
     */
    public Pair<Boolean, Boolean> areServosTurned() {
        return new Pair<>(frontServo.getAngle() == RELEASE_ANGLE, backServo.getAngle() == RELEASE_ANGLE);
    }
}