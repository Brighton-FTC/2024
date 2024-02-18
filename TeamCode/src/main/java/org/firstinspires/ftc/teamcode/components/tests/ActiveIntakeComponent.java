package org.firstinspires.ftc.teamcode.components.tests;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
public class ActiveIntakeComponent {
        private boolean motorState = false;
        private double motorSpeed = 0.9;
        private int servoAngle = 0;
        DcMotor dcMotor = hardwareMap.dcMotor.get("motor");
        Servo servo = hardwareMap.servo.get("left_servo");
        Servo servo = harwareMap.servo.get("right_servo");


        public void turnMotorOn() {
            motorState = true;
            setVelocity(motorSpeed);
        }
        public void turnMotorOff() {
            motorState = false;
            setVelocity(double 0);
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
            servo.rotateByAngle(180);
            servo.rotateByAngle(-180);
        }
        public void rotateServosManually() {
            servoAngle += 180;
        }
    }
}
