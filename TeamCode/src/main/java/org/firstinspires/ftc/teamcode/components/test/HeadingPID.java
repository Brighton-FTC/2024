package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;

public class HeadingPID {

    private BCGyro gyro;

    private double desiredHeading;

    // TODO: Tune this
    private static final double turnConstant = 0;
    private final PIDController pid = new PIDController(0, 0, 0);


    public HeadingPID(BCGyro gyro){
        this.gyro = gyro;
    }

    /**
     * Does PID calculations and returns the value to add to the turnSpeed parameter in order to correct heading
     *
     * @param turnSpeed the turnSpeed passed to the MecanumDrive, gotten from the gamepad
     * @param time in millseconds
     * @return
     */
    public double runPID(double turnSpeed, double time){
        desiredHeading += turnSpeed * turnConstant;
        pid.setSetPoint(desiredHeading);

        return pid.calculate(gyro.getHeading());
    }
}
