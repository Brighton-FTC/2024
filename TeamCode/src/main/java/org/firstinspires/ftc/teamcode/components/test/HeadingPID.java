package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private IMU gyro;

    private double desiredHeading = 0;

    // TODO: Tune this
    private double turnConstant = 900;
    // TODO: Tune this
    private final PIDController pid = new PIDController(0.05, 0, 0.0032);

    private Telemetry telemetry;

//    private MecanumDriveOdometry odo;


    public HeadingPID(Telemetry telemetry, IMU gyro){
//        odo = new MecanumDriveOdometry();
        this.gyro = gyro;
        this.telemetry = telemetry;
    }

    private void resetDesiredHeading(){
        desiredHeading = desiredHeading < 0 ? desiredHeading + 360 : desiredHeading;
        desiredHeading = desiredHeading > 360 ? desiredHeading - 360 : desiredHeading;
    }

    public void setTurnConstant(double newConstant){
        turnConstant = newConstant;
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
        double currentHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        resetDesiredHeading();
        if (currentHeading < 0){
            currentHeading *= -1 ;
        }
        else {
            currentHeading = 360 - currentHeading;
        }
        pid.setSetPoint(desiredHeading);
        double difference = Math.abs(desiredHeading - currentHeading);
        double modifiedHeading = currentHeading;
        if (difference > 180){
            int sign = desiredHeading > currentHeading ? 1 : -1;
            modifiedHeading = desiredHeading + sign * (360 - difference);
        }
        telemetry.addData("Current heading ", currentHeading);
        telemetry.addData("Modified heading ", modifiedHeading);
        telemetry.addData("Desired heading ", desiredHeading);
        double output = pid.calculate(modifiedHeading);
        return output;
    }
}
