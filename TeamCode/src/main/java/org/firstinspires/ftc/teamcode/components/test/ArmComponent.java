package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingMotorEx;

/**
 * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
 */
public class ArmComponent {

//    public static final int GRABBER_ROTATE_DOWN_POSITION = -60;
//    public static final int GRABBER_ROTATE_UP_POSITION = 220;

    // TODO: fill in these values
    public static final int ARM_LIFTED_POSITION = 0;
    public static final int ARM_LOWERED_POSITION = 2000;

    private static final int f = 0;

    private final FTCLibCachingMotorEx armMotor;
    private boolean isArmLifted = false;

    // TODO: Tune this
    private final PIDController pid = new PIDController(0, 0, 0);

    // we are using hd on arm yes
    // got this from LRR driveconstants page
    private final double ticks_in_degrees = 560.0 / 360.0;

    private double currentVelocity;

    private double currentPosition;

    /**
     * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
     *
     * @param armMotor The motor that controls the arm.
     */
    public ArmComponent(@NonNull FTCLibCachingMotorEx armMotor) {
        this.armMotor = armMotor;
        setTargetPosition(ARM_LOWERED_POSITION);
    }

    /**
     * Call once to set the arm to be lifted. <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void lift() {
        isArmLifted = true;

        setTargetPosition(ARM_LIFTED_POSITION);
    }

    /**
     * Call once to set the arm to be lowered. <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void lower() {
        isArmLifted = false;

        setTargetPosition(ARM_LOWERED_POSITION);
    }

    /**
     * Call once to toggle the position of the arm <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void toggle() {
        if (isArmLifted) {
            lower();
        } else {
            lift();
        }

        isArmLifted = !isArmLifted;
    }


    /**
     * Directly control the movement of the arm.
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
    public boolean isLifted() {
        return isArmLifted;
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
        armMotor.set(pid.calculate(currentPosition) + ff);
    }

    /**
     * Get the setpoint of the PID controller.
     * @return The setpoint of the PID Controller, in ticks.
     */
    public double getSetPoint(){
        return pid.getSetPoint();
    }

    /**
     * Get if the motor is at the setpoint.
     * @return a boolean for if the motor is at the setpoint.
     */
    public boolean atSetPoint(){
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
    public void read(){
        currentPosition = armMotor.getCurrentPosition();
        currentVelocity = armMotor.getVelocity();
    }
}