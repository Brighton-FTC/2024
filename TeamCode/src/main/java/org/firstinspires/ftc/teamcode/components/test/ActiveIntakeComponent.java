package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Component class for active intake. <br />
 * Call {@link #turnContinually()} or {@link #turnManually()} once and then call {@link #moveMotor()} continuously to move the motor.
 */
public class ActiveIntakeComponent {
    public static final double MOTOR_SPEED = 0.5;
    public static final int TURN_MANUALLY_DEGREES = 270;

    private State state = State.OFF;
    private final double DEGREES_PER_TICK = 360.0 / 560.0;
    private final MotorEx motor1, motor2;

    /**
     * Component class for active intake:
     */
    public ActiveIntakeComponent(MotorEx motor1, MotorEx motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        this.motor1.setRunMode(Motor.RunMode.VelocityControl);
        this.motor2.setRunMode(Motor.RunMode.VelocityControl);

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
            if (motor1.getCurrentPosition() >= TURN_MANUALLY_DEGREES / DEGREES_PER_TICK
                    && motor2.getCurrentPosition() >= TURN_MANUALLY_DEGREES / DEGREES_PER_TICK) {
                state = State.OFF;
            } else {
                motor1.set(MOTOR_SPEED);
                motor2.set(MOTOR_SPEED);
            }
        } else if (state == State.TURNING_CONTINUOUSLY) {
            motor1.set(MOTOR_SPEED);
            motor2.set(MOTOR_SPEED);
        } else {
            motor1.set(0);
            motor2.set(0);
        }
    }

    public State getState() {
        return state;
    }

    public MotorEx[] getMotors() {
        return new MotorEx[] {motor1, motor2};
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