package org.firstinspires.ftc.teamcode.components.test;

import static java.lang.Math.max;
import static java.lang.Math.min;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;


/**
 * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
 */
@Config

public class ArmComponent {

    public static final double PICKUP_ARM_POS = 0;
    private final MotorEx armMotor;

    private State state = State.PICKUP_GROUND;

    // TODO: Tune this
    private final PIDController pid = new PIDController(0.008, 0.0005, 0.0005);

    private double currentVelocity;

    private double currentPosition;

    private static final double max_power = 0.5;

    /**
     * Code to lift/lower arm. Also tilts the grabber (up/down) when arm is lifted or lowered.
     *
     * @param armMotor The motor that controls the arm.
     */
    public ArmComponent(@NonNull MotorEx armMotor) {
        this.armMotor = armMotor;
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.armMotor.resetEncoder();
        setTargetPosition(State.PICKUP_GROUND.position);
    }

    /**
     * Call once to set the arm to a certain state. <br />
     * (You need to call {@link #moveToSetPoint()} for the arm to actually move.
     */
    public void setState(@NonNull State newState) {
        state = newState;

//        outtakeRotationServo.setPosition(newState.rotationAngle);
        setTargetPosition(state.position);
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
        double power = pid.calculate(currentPosition);

        power = min(max_power, power);
        power = max(-max_power, power);
        armMotor.set(power);
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

    public void pickup() {
        pid.setSetPoint(PICKUP_ARM_POS);
    }

    public MotorEx getArmMotor() {
        return armMotor;
    }

    public Action goToStateAction(State state) {
        return new Action() {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    setState(state);
                    init = true;
                }

                read();
                moveToSetPoint();

                return !atSetPoint();
            }
        };
    }

    @Config
    public enum State {
        PICKUP_GROUND(0),
        PLACE_LOW_BACKDROP(-700),
        PLACE_HIGH_BACKDROP(-800),
        PLACE_GROUND(-1100);

        public final int position;

        State(int position) {
            this.position = position;
        }
    }
}