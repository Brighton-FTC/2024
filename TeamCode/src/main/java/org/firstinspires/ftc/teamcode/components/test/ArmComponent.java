package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;


/**
 * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
 */
public class ArmComponent {
    // TODO: fill in these values
    private static final int f = 0;

    private final MotorEx armMotor;
    private State state = State.GROUND;

    // TODO: Tune this
    private final PIDController pid = new PIDController(0, 0, 0);

    // we are using hd on arm yes
    // got this from LRR drive constants page
    public final double ticks_in_degrees = 560.0 / 360.0;

    private double currentVelocity;

    private double currentPosition;

    // TODO: Replace this with the voltage that we tuned the arm at.
    private final double tuningVoltage = 12.;

    private final double voltageNormalization;

    /**
     * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
     *
     * @param armMotor The motor that controls the arm.
     */
    public ArmComponent(@NonNull MotorEx armMotor, double currentVoltage) {
        voltageNormalization = currentVoltage / tuningVoltage;
        this.armMotor = armMotor;
        setTargetPosition(State.GROUND.position);
    }

    /**
     * Call once to set the arm to a certain state. <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void setState(State newState) {
        state = newState;

        setTargetPosition(state.position);
    }

    /**
     * Call once to toggle the position of the arm <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void toggle() {
        if (state == State.GROUND) {
            setState(State.HIGH);

        } else {
            setState(State.GROUND);
        }
    }


    /**
     * Directly control the movement of the arm.
     *
     * @param velocity The velocity (-1 to 1) that the motor is set to.
     */
    public void setVelocity(double velocity) {
        armMotor.set(velocity);
    }

    /**
     * Get the state of the arm (lifted/lowered).
     *
     * @return If the arm is lifted or not.
     */
    public State getState() {
        return state;
    }

    /**
     * Get the position of the arm (in ticks).
     *
     * @return The position of the arm.
     */
    public double getArmPosition() {
        return currentPosition;
    }

    /**
     * Get the velocity of the arm (in ticks per second).
     *
     * @return The velocity of the arm motor.
     */
    public double getArmVelocity() {
        return currentVelocity;
    }

    /**
     * Call continuously to move the arm to the required position.
     */
    public void moveToSetPoint() {
        double ff = Math.cos(Math.toRadians(pid.getSetPoint() / ticks_in_degrees)) * f;

        // essentially, voltage normalization is adjusting the motor input for the current voltage supplied by the battery
        armMotor.set((pid.calculate(currentPosition) + ff) / voltageNormalization);
    }

    /**
     * Get the setpoint of the PID controller.
     *
     * @return The setpoint of the PID Controller, in ticks.
     */
    public double getSetPoint() {
        return pid.getSetPoint();
    }

    /**
     * Get if the motor is at the setpoint.
     *
     * @return a boolean for if the motor is at the setpoint.
     */
    public boolean atSetPoint() {
        return pid.atSetPoint();
    }


    /**
     * Set PID and feedforward to desired position
     *
     * @param position The desired final position of the arm
     */
    private void setTargetPosition(int position) {
        pid.setSetPoint(position);
    }

    /**
     * Reads from motors and stores data in ArmComponent.
     * Call in every loop or encoders break.
     */
    public void read() {
        currentPosition = armMotor.getCurrentPosition();
        currentVelocity = armMotor.getVelocity();
    }

    public enum State {
        // TODO: TUNE THIS
        // also this starts from 1 because yay
        GROUND(2000, 0), // nothing actually leads to ground because ground is separate
        LOW(-500, 1),
        MIDDLE(-1000, 2),
        HIGH(0, 3);

        public final int position;

        public final int index;

        State(int position, int index) {
            this.position = position;
            this.index = index;
        }

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GROUND:
                    return "GROUND";
                case LOW:
                    return "LOW";
                case MIDDLE:
                    return "MIDDLE";
                case HIGH:
                    return "HIGH";
                default:
                    return "UNKNOWN";
            }
        }
    }
}