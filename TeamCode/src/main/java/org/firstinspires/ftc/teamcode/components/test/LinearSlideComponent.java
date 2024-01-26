package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingMotorEx;

/**
 * Linear slide component. <br />
 *
 * Call {@link #read()} every loop or the getters will break.
 * Call {@link #lift()} or {@link #lower()} to set the linear slide to be lifted/lowered. <br />
 * Call {@link #moveToSetPoint()} continuously to move the linear slide to be lifted/lowered.
 */
public class LinearSlideComponent {
    // TODO: fill in
    public static final int LINEAR_SLIDE_LIFTED_POSITION = -1800;
    public static final int LINEAR_SLIDE_LOWERED_POSITION = -100;

    private final FTCLibCachingMotorEx linearSlideMotor;
    private final PIDFController pid;

    private double currentVelocity;

    private double currentPosition;

    private boolean isLifted = false;

    private final ArmComponent arm;
    private static final double f = 0;

    public final double ticks_in_degrees = 288.0 / 360.0;

    // TODO: replace with the voltage during tuning
    private final double tuningVoltage = 12.;
    private final double voltageNormalization;

    /**
     * Linear slide component.
     * @param linearSlideMotor The motor that controls the linear slide.
     */
    public LinearSlideComponent(FTCLibCachingMotorEx linearSlideMotor, ArmComponent arm, double currentVoltage) {
        voltageNormalization = currentVoltage / tuningVoltage;
        this.linearSlideMotor = linearSlideMotor;
        this.arm = arm;

        pid = new PIDController(0.1, 0.01, 0.01);
        pid.setTolerance(3);
    }

    /**
     * Set the linear slide to be lifted. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lift() {
        pid.setSetPoint(LINEAR_SLIDE_LIFTED_POSITION);

        isLifted = true;
    }

    /**
     * Set the linear slide to be lowered. <br />
     * You need to call {@link #moveToSetPoint()} for the linear slide to actually move.
     */
    public void lower() {
        pid.setSetPoint(LINEAR_SLIDE_LOWERED_POSITION);

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
        double ff = Math.sin(Math.toRadians(arm.getArmPosition() / arm.ticks_in_degrees)) * f;

        // voltage normalization is essentially adjusting motor input according to current voltage
        // current voltage can be different to tuning votlage, causes problems
        linearSlideMotor.set((pid.calculate(currentPosition) + ff) / voltageNormalization);

        if (pid.atSetPoint()){
            linearSlideMotor.set(0.05);
        }
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
    public boolean isLifted() {
        return isLifted;
    }

    /**
     * Get if the linear slide is at its set point.
     * @return If the linear slide is at its set point.
     */
    public boolean atSetPoint() {
        return pid.atSetPoint();
    }

    /**
     * Get the position of the linear slide motor.
     * @return The position of the linear slide motor, in ticks.
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * Get the velocity of the linear slide motor.
     * @return The velocity of the linear slide motor, in ticks per second.
     */
    public double getVelocity() {
        return currentVelocity;
    }

    /**
     * Read from the motor.
     * MUST be called every loop, or getters won't work at all.
     */
    public void read(){
        currentVelocity = linearSlideMotor.getVelocity();
        currentPosition = linearSlideMotor.getCurrentPosition();
    }

    /**
     * Get the setpoint of the PID controller.
     * @return The setpoint of the PID Controller, in ticks.
     */
    public double getSetPoint(){
        return pid.getSetPoint();
    }
}