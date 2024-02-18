package org.firstinspires.ftc.teamcode.components.tests;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ActiveIntakeComponent {
    private boolean isMotorOn = false;
    private final double MOTOR_SPEED = 0.9;
    private int servoAngle = 0;
    private MotorEx motor;
    private ServoEx leftServo;
    private ServoEx rightServo;

    public ActiveIntakeComponent(MotorEx motor, ServoEx leftServo, ServoEx rightServo) {
        this.motor = motor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
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

    public void toggleMotor() {
        if (isMotorOn) {
            turnMotorOff();
        } else {
            turnMotorOn();
        }
    }

    public void rotateServosForStack(int pixels) {
        leftServo.rotateByAngle(180 * pixels);
        rightServo.rotateByAngle(-180 * pixels);
    }

    public void rotateServosManually() {
        leftServo.rotateByAngle(180);
        rightServo.rotateByAngle(-180);
    }
}
