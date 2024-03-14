package org.firstinspires.ftc.teamcode.components.test.heading;

import com.acmerobotics.dashboard.FtcDashboard;
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
@Autonomous(name = "Heading PID Tuner", group = "tuner")
public class HeadingPIDTuner extends OpMode {

    private BCGyro gyro;

    private static double desiredHeading;

    // TODO: Tune this
    private static double turnConstant = 0;
    // TODO: Tune this
    private PIDController pid = new PIDController(0, 0, 0);

    private static double kp = 0;
    private static double ki = 0;
    private static double kd = 0;

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
        pid.calculate(currentHeading);

        drive.driveRobotCentric(0, pid.calculate(currentHeading), 0);
        telemetry.addData("Current heading ", currentHeading);
        telemetry.addData("Desired heading ", desiredHeading);
    }
}
