package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Component class for active intake. <br />
 * Call {@link #turnContinually()} or {@link #turnManually()} once and then call {@link #moveMotor()} continuously to move the motor.
 */
public class ActiveIntakeComponent {
    private State state;
    private final double DEGREES_PER_TICK = 360 / 560;
    private final MotorEx motor;

    /**
     * Component class for active intake:
     *
     * @param motor The active intake motor.
     */
    public ActiveIntakeComponent(MotorEx motor) {
        this.motor = motor;
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
            motor.resetEncoder();
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
            motor.set();
            if (motor.getCurrentPosition() >= 180 / DEGREES_PER_TICK) {
                state = State.OFF;
            } else {
                motor.set(1);

            }
        } else if (state == State.TURNING_CONTINUOUSLY) {
            motor.set(1);
        }
    }

    /**
     * The state that the active intake is in.
     */
    public enum State {
        OFF,
        TURNING_MANUALLY,
        TURNING_CONTINUOUSLY
    }
}