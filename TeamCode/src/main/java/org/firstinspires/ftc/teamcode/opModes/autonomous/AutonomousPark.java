package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "park")
public class AutonomousPark extends OpMode {
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;
    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;

    @Override
    public void init() {
        frontLeftMotor = new MotorEx(hardwareMap,"front_left_drive");
        frontRightMotor = new MotorEx(hardwareMap,"front_right_drive");
        backLeftMotor = new MotorEx(hardwareMap,"back_left_drive");
        backRightMotor = new MotorEx(hardwareMap,"back_right_drive");
    }

    @Override
    public void loop() {
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        frontLeftMotor.set(0.3);
        frontRightMotor.set(0.3);
        backLeftMotor.set(0.3);
        backRightMotor.set(0.3);
    }
}
