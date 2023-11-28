package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
 */
public class ArmComponent {
    // TODO: fill in these values
    public static final int GRABBER_TILT_DOWN_POSITION = 0;
    public static final int GRABBER_TILT_UP_POSITION = 180;
    public static final int ARM_LIFTED_POSITION = 0;
    public static final int ARM_LOWERED_POSITION = 2000;

    private final ServoEx grabberTiltServo;

    private final MotorEx armMotor;
    private boolean isArmLifted = false;

    // TODO: Tune this
    private final PIDFController pidf = new PIDFController(0, 0, 0, 0);

    /**
     * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
     *
     * @param armMotor         The motor that controls the arm.
     * @param grabberTiltServo The servo that controls the grabber tilting.
     */
    public ArmComponent(@NonNull MotorEx armMotor, @NonNull ServoEx grabberTiltServo) {
        this.armMotor = armMotor;
        this.grabberTiltServo = grabberTiltServo;

        grabberTiltServo.setRange(GRABBER_TILT_DOWN_POSITION, GRABBER_TILT_UP_POSITION);

        setTargetPosition(ARM_LOWERED_POSITION);
        grabberTiltServo.turnToAngle(GRABBER_TILT_DOWN_POSITION);
    }

    /**
     * Call once to set the arm to be lifted. <br />
     * (You need to call {@link #moveToSetPosition()} for the arm to actually move.
     */
    public void lift() {
        isArmLifted = true;

        setTargetPosition(ARM_LIFTED_POSITION);

        grabberTiltServo.turnToAngle(GRABBER_TILT_UP_POSITION);
        armMotor.set(pidf.calculate(armMotor.getCurrentPosition()));
    }

    /**
     * Call once to set the arm to be lowered. <br />
     * (You need to call {@link #moveToSetPosition()} for the arm to actually move.
     */
    public void lower() {
        isArmLifted = false;

        setTargetPosition(ARM_LOWERED_POSITION);

        grabberTiltServo.turnToAngle(GRABBER_TILT_DOWN_POSITION);
        armMotor.set(pidf.calculate(armMotor.getCurrentPosition()));
    }

    /**
     * Call once to toggle the position of the arm <br />
     * (You need to call {@link #moveToSetPosition()} for the arm to actually move.
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
        return armMotor.getCurrentPosition();
    }

    /**
     * Get the velocity of the arm (in ticks per second).
     *
     * @return The velocity of the arm motor.
     */
    public double getArmVelocity() {
        return armMotor.getVelocity();
    }

    /**
     * Get the angle that the grabber is tilted at (in degrees).
     *
     * @return The angle of the grabber tilt servo.
     */
    public double getGrabberTiltAngle() {
        return grabberTiltServo.getAngle();
    }

    /**
     * Call continuously to move the arm to the required position.
     */
    public void moveToSetPosition() {
        armMotor.set(pidf.calculate(armMotor.getCurrentPosition()));
    }

    /**
     * Set PID and feedforward to desired position
     *
     * @param position The desired final position of the arm
     */
    private void setTargetPosition(int position) {
        pidf.setSetPoint(position);
    }
}