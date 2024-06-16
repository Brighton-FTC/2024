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
    // TODO: fill in these values

    private final ServoEx backOuttakeServo;
    private final ServoEx frontOuttakeServo;

    private boolean isOuttakeClosed = true;

    /**
     * Code to open/close outtake, and tilt outtake. <br />
     *
     * @param frontOuttakeServo The servo at the front of the outtake (nearest the intake).
     * @param backOuttakeServo  The servo at the back of the outtake (nearest the arm).
     */
    public OuttakeComponent(ServoEx frontOuttakeServo, ServoEx backOuttakeServo) {
        this.frontOuttakeServo = frontOuttakeServo;
        this.frontOuttakeServo.setRange(0, 90);
        this.backOuttakeServo = backOuttakeServo;
        this.backOuttakeServo.setRange(0, 90);
    }

    public void toggleFrontOuttake(){
        if (frontOuttakeServo.getPosition() == 0) {
            releaseFront();
        } else {
            holdFront();
        }
    }

    public void releaseFront(){
        frontOuttakeServo.turnToAngle(90);
    }
    public void releaseBack(){
        backOuttakeServo.turnToAngle(90);
    }

    public void releaseAll(){
        releaseFront();
        releaseBack();
    }

    public void holdFront(){
        frontOuttakeServo.turnToAngle(0);
    }
    public void holdBack(){
        backOuttakeServo.turnToAngle(0);
    }

    public void holdAll(){
        holdFront();
        holdBack();
    }

    public void toggleBackOuttake() {
        if (backOuttakeServo.getPosition() == 0) {
            releaseBack();
        } else {
            holdBack();
        }
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

    public double getFrontServoPosition(){
        return frontOuttakeServo.getPosition();
    }

    public double getBackServoPosition(){
        return backOuttakeServo.getPosition();
    }
}