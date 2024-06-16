package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Code to open/close outtake, and tilt outtake. <br />
 */

public class OuttakeComponent {
    // TODO: fill in these values

    private final ServoEx backOuttakeServo;
    private final ServoEx frontOuttakeServo;

    private boolean isOuttakeClosed = true;


    /**
     * Code to open/close outtake, and tilt outtake. <br />
     * @param outtakeServo The servo that controls the outtake.
     */
    public OuttakeComponent(ServoEx frontOuttakeServo, ServoEx backOuttakeServo) {
        this.frontOuttakeServo = frontOuttakeServo;
        this.frontOuttakeServo.setRange(0, 360);
        this.backOuttakeServo = frontOuttakeServo;
        this.backOuttakeServo.setRange(0, 360);
    }

    public void toggleFrontOuttake(){
        if (frontOuttakeServo.getPosition() == 0) {
            frontOuttakeServo.turnToAngle(90);
        } else {
            frontOuttakeServo.turnToAngle(0);
        }
    }

    public void toggleBackOuttake(){
        if (backOuttakeServo.getPosition() == 0) {
            backOuttakeServo.turnToAngle(90);
        } else {
            backOuttakeServo.turnToAngle(0);
        }
    }
}