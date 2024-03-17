package org.firstinspires.ftc.teamcode.components.test.heading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;

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

@Config
@Autonomous(name = "Heading PID Tuner", group = "tuner")
public class HeadingPIDTuner extends OpMode {

    // P = 0.05
    // I = 0
    // D = 0.0032

    private BCGyro gyro;

    public static double desiredHeading;

    // TODO: Tune this
    private static double turnConstant = 0;
    // TODO: Tune this
    private PIDController pid = new PIDController(0, 0, 0);

    public static double kp = 0.05;
    public static double ki = 0.001;
    public static double kd = 0.0033;

    private static double currentHeading = 0;

    private MecanumDrive drive;

    @Override
    public void init() {
        this.gyro = new BCGyro(hardwareMap);
        gyro.init();

        drive = new MecanumDrive(new MotorEx(hardwareMap, "front_left_drive"),
                                 new MotorEx(hardwareMap, "front_right_drive"),
                                 new MotorEx(hardwareMap, "back_left_drive"),
                                 new MotorEx(hardwareMap, "back_right_drive"));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pid.setPID(kp, ki, kd);
        currentHeading = gyro.getHeading();
        pid.setSetPoint(desiredHeading);
        double difference = Math.abs(desiredHeading - currentHeading);
        double modifiedHeading = currentHeading;
        double value;
        if (difference > 180){
            value = 360;
            int sign = desiredHeading > currentHeading ? 1 : -1;
            modifiedHeading = desiredHeading + sign * (360 - difference);
        }
        else {
            value = 0;
        }
        telemetry.addData("value ", value);
        double output = pid.calculate(modifiedHeading);


        drive.driveRobotCentric(0, 0, output);
        telemetry.addData("Current heading ", currentHeading);
        telemetry.addData("Output: ", output);
        telemetry.addData("Modified heading ", modifiedHeading);
        telemetry.addData("Desired heading ", desiredHeading);
    }
}
