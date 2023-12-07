package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Linear slide component. <br />
 * Call {@link #lift()} or {@link #lower()} to set the linear slide to be lifted/lowered. <br />
 * Call {@link #moveToSetPoint()} continuously to move the linear slide to be lifted/lowered.
 */
public class LinearSlideComponent {
    public static final int ERROR = 10;
    public static final double VELOCITY = 0.5;

    // TODO: fill in
    public static final int LINEAR_SLIDE_LIFTED_POSITION = -1800;
    public static final int LINEAR_SLIDE_LOWERED_POSITION = -100;

    private final MotorEx linearSlideMotor;

    private int target = LINEAR_SLIDE_LOWERED_POSITION;
    private boolean isLifted = false;

    /**
     * Linear slide component.
     *
     * @param linearSlideMotor The motor that controls the linear slide.
     */
    public LinearSlideComponent(MotorEx linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
    }

    /**
     * Set the linear slide to be lifted. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lift() {
        target = LINEAR_SLIDE_LIFTED_POSITION;

        isLifted = true;
    }

    /**
     * Set the linear slide to be lowered. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lower() {
        target = LINEAR_SLIDE_LIFTED_POSITION;

        isLifted = false;
    }

    /**
     * Lower the linear slide if it is lifted, and lift it if it is lowered. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void toggle() {
        if (isLifted) {
            lower();
        } else {
            lift();
        }
    }

    /**
     * Move the linear slide to the specified set point.
     */
    public void moveToSetPoint() {
        if (!atSetPoint()) {
            if (linearSlideMotor.getCurrentPosition() < target) {
                linearSlideMotor.set(VELOCITY);
            } else {
                linearSlideMotor.set(-VELOCITY);
            }
        }
    }

    /**
     * Directly control the movement of the linear slide.
     *
     * @param velocity The velocity (-1 to 1) that the motor is set to.
     */
    public void setVelocity(double velocity) {
        linearSlideMotor.set(velocity);
    }

    /**
     * Get the position of the linear slide.
     *
     * @return If the linear slide is lowered or not.
     */
    public boolean isLifted() {
        return isLifted;
    }

    /**
     * Get if the linear slide is at its set point.
     *
     * @return If the linear slide is at its set point.
     */
    public boolean atSetPoint() {
        return Math.abs(target - linearSlideMotor.getCurrentPosition()) <= ERROR;
    }

    /**
     * Get the position of the linear slide motor.
     *
     * @return The position of the linear slide motor, in ticks.
     */
    public double getPosition() {
        return linearSlideMotor.getCurrentPosition();
    }

    /**
     * Get the velocity of the linear slide motor.
     *
     * @return The velocity of the linear slide motor, in ticks per second.
     */
    public double getVelocity() {
        return linearSlideMotor.getVelocity();
    }

    /**
     * Get the setpoint of the PID controller.
     *
     * @return The setpoint of the motor, in ticks.
     */
    public double getSetPoint() {
        return target;
    }
}