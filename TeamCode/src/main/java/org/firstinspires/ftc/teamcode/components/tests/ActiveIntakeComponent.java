package org.firstinspires.ftc.teamcode.components.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ActiveIntakeComponent {
    private boolean motorState = false;
    private double motorSpeed = 0.9;
    private int servoAngle = 0;
    MotorEx motor = new MotorEx(hardwareMap, "motor");
    ServoEx left_servo = new SimpleServo(hardwareMap, "left_servo", 0, 360);
    ServoEx right_servo = new SimpleServo(hardwareMap, "right_servo", 0, 360);



    public void turnMotorOn() {
        motorState = true;
        motor.setVelocity(motorSpeed);
    }
    public void turnMotorOff() {
        motorState = false;
        setVelocity(0);
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
        left_servo.rotateByAngle(180);
        right_servo.rotateByAngle(-180);
    }
    public void rotateServosManually() {
        servoAngle += 180;
    }
}
