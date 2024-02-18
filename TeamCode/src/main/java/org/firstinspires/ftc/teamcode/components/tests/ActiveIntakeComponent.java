package org.firstinspires.ftc.teamcode.components.tests;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ActiveIntakeComponent {
    private boolean motorState = false;
    private double motorSpeed = 0.9;
    private int servoAngle = 0;
    MotorEx motor;
    ServoEx leftServo;
    ServoEx rightServo;

    public ActiveIntakeComponent(MotorEx motor, ServoEx leftServo, ServoEx rightServo) {
        this.motor = motor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }


    public void turnMotorOn() {
        motorState = true;
        motor.set(motorSpeed);
    }

    public void turnMotorOff() {
        motorState = false;
        motor.set(0);
    }

    public void toggleMotor() {
        if (motorState) {
            turnMotorOff();
        } else {
            turnMotorOn();
        }
    }

    public void rotateServosForStack(int pixels) {
        servoAngle = 180 * pixels;
        leftServo.rotateByAngle(180);
        rightServo.rotateByAngle(-180);
    }

    public void rotateServosManually() {
        servoAngle += 180;
    }
}
