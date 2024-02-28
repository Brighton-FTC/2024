package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ActiveIntakeComponent {
    private boolean isMotorOn = false;
    private final double DEGREES_PER_TICK = 360/560;
    private final double MOTOR_SPEED = 0.9;
    private final MotorEx motor;

    public ActiveIntakeComponent(MotorEx motor) {
        this.motor = motor;
    }

    public boolean isMotorOn() {
        return isMotorOn;
    }

    public void turnMotorOn() {
        isMotorOn = true;
        motor.set(MOTOR_SPEED);
    }

    public void turnMotorOff() {
        isMotorOn = false;
        motor.set(0);
    }

    public void turnMotorManually() {

    }
}