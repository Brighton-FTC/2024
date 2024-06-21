package org.firstinspires.ftc.teamcode.components.test;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
<<<<<<< HEAD
 * Component class for active intake. <br />.
=======
 * Component class for active intake. <br />
 * Call {@link #turnContinually()} to continuously move the motor, and {@link #turnMotorOff()} to turn off.
>>>>>>> teleop
 */
public class ActiveIntakeComponent {
    private final MotorEx intakeMotor;
    private boolean isTurning = false;

    /**
     * Component class for active intake:
     */
    public ActiveIntakeComponent(MotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    /**
     * Turns motor off.
     */
    public void turnMotorOff() {
        isTurning = false;
        this.intakeMotor.set(0);
    }

    /**
     * Turns motor continuously
     */
    public void turnForwards() {
        isTurning = true;
        this.intakeMotor.set(1);
    }

    public void turnBackwards(){
        isTurning = true;
        this.intakeMotor.set(-1);
    }

    public boolean isTurning() {
        return isTurning;
    }

    public MotorEx getMotor() {
        return intakeMotor;
    }

    public Action turnForwardsAction() {
        return telemetryPacket -> {
            turnForwards();
            return false;
        };
    }

    public Action turnMotorOffAction() {
        return telemetryPacket -> {
            turnMotorOff();
            return false;
        };
    }
}