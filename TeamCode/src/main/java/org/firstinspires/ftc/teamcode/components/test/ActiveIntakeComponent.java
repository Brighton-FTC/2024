package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Component class for active intake. <br />
 * Call {@link #turnContinually()} or {@link #turnManually()} once and then call {@link #moveMotor()} continuously to move the motor.
 */
public class ActiveIntakeComponent {
    private State state;
    private final double DEGREES_PER_TICK = 360.0 / 560.0;
    private final MotorEx motor1, motor2;

    /**
     * Component class for active intake:
     */
    public ActiveIntakeComponent(MotorEx motor1, MotorEx motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        this.motor2.setInverted(true);
    }

    /**
     * Turns motor off.
     */
    public void turnMotorOff() {
        state = State.OFF;
    }

    /**
     * Turns motor 180 degrees manually
     */
    public void turnManually() {
        if (state == State.OFF) {
            state = State.TURNING_MANUALLY;
            motor1.resetEncoder();
            motor2.resetEncoder();
        }
    }

    /**
     * Turns motor continuously
     */
    public void turnContinually() {
        state = State.TURNING_CONTINUOUSLY;
    }

    /**
     * Call in loop to actually move the motor.
     */
    public void moveMotor() {
        if (state == State.TURNING_MANUALLY) {
            if (motor1.getCurrentPosition() >= 180 / DEGREES_PER_TICK
                    && motor2.getCurrentPosition() >= 180 / DEGREES_PER_TICK) {
                state = State.OFF;
            } else {
                motor1.set(1);
                motor2.set(1);
            }
        } else if (state == State.TURNING_CONTINUOUSLY) {
            motor1.set(1);
            motor2.set(1);
        }
    }

    public State getState() {
        return state;
    }

    /**
     * The state that the active intake is in.
     */
    public enum State {
        OFF,
        TURNING_MANUALLY,
        TURNING_CONTINUOUSLY;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case OFF:
                    return "OFF";
                case TURNING_MANUALLY:
                    return "TURNING MANUALLY";
                case TURNING_CONTINUOUSLY:
                    return "TURNING CONTINUOUSLY";
                default:
                    return "UNKNOWN";
            }
        }
    }
}