package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Linear slide component. <br />
 * Call {@link #lift()} or {@link #lower()} to set the linear slide to be lifted/lowered. <br />
 * Call {@link #moveToSetPoint()} continuously to move the linear slide to be lifted/lowered.
 */
public class LinearSlideComponent {
    // TODO: fill in
    public static final int LINEAR_SLIDE_LIFTED_POSITION = 2000;
    public static final int LINEAR_SLIDE_LOWERED_POSITION = 0;

    private final MotorEx linearSlideMotor;
    private final PIDFController pidf;

    private boolean isLowered = true;

    /**
     * Linear slide component.
     * @param linearSlideMotor The motor that controls the linear slide.
     */
    public LinearSlideComponent(MotorEx linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;

        pidf = new PIDFController(0, 0, 0, 0); // TODO: tune this
    }

    /**
     * Set the linear slide to be lifted. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lift() {
        pidf.setSetPoint(LINEAR_SLIDE_LIFTED_POSITION);

        isLowered = false;
    }

    /**
     * Set the linear slide to be lowered. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lower() {
        pidf.setSetPoint(LINEAR_SLIDE_LOWERED_POSITION);

        isLowered = true;
    }

    /**
     * Lower the linear slide if it is lifted, and lift it if it is lowered. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void toggle() {
        if (isLowered) {
            lift();
        } else {
            lower();
        }
    }

    /**
     * Move the linear slide to the specified set point.
     */
    public void moveToSetPoint() {
        linearSlideMotor.set(pidf.calculate(linearSlideMotor.getCurrentPosition()));
    }

    /**
     * Directly control the movement of the linear slide.
     * @param velocity The velocity (-1 to 1) that the motor is set to.
     */
    public void setVelocity(double velocity) {
        linearSlideMotor.set(velocity);
    }

    /**
     * Get the position of the linear slide.
     * @return If the linear slide is lowered or not.
     */
    public boolean isLowered() {
        return isLowered;
    }
}