package org.firstinspires.ftc.teamcode.components.test.heading;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This class corrects for our drivetrain curving.
 * Essentially, our (mecanum) drivetrain has minor problems driving straight and major problems strafing
 * This is due to the 90 degree gearboxes being only on the front motors. (poverty yay!)
 *
 * To fix this, we use Heading PID to get a value to add to the turnSpeed.
 * The setpoint is what heading the drivetrain SHOULD be at (based on turn inputs from gamepad).
 * This is saved in desiredHeading.
 * Then, the actual heading of the bot (based on gyro) is passed into the controller, and a value is returned!
 */
public class HeadingPID {

    private IMU imu;

    private double desiredHeading;

    // TODO: Tune this
    private static final double turnConstant = 0;
    // TODO: Tune this
    private final PIDController pid = new PIDController(0, 0, 0);

//    private MecanumDriveOdometry odo;


    public HeadingPID(IMU imu){
        this.imu = imu;
    }

    /**
     * Does PID calculations - call this every loop and add the result to turnSpeed of drivetrain.
     *
     * @param turnSpeed the turnSpeed passed to the MecanumDrive, gotten from the gamepad
     * @param time in seconds after last time this method was called
     * @return the value to add to the turnSpeed parameter in order to correct heading
     */
    public double runPID(double turnSpeed, double time){
        desiredHeading += turnSpeed * turnConstant * time;
        pid.setSetPoint(desiredHeading);

        return pid.calculate(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)); // TODO: see if the previously tuned values require degrees or radians
    }
}
