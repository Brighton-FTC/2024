package org.firstinspires.ftc.teamcode.components.test.heading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
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
@Autonomous(name = "Heading PID Tester", group = "tuner")
public class HeadingPIDTester extends OpMode {

    private IMU gyro;

    private MecanumDrive drive;

    private ElapsedTime time;

    private HeadingPID headingPID;
    private GamepadEx gamepad;

    private final double DEAD_ZONE_SIZE = 0.2;

    private double previousTurning;

    public static double turnConstant = 1044;

    @Override
    public void init() {
        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );
        gyro.resetYaw();

        drive = new MecanumDrive(new MotorEx(hardwareMap, "front_left_drive"),
                                 new MotorEx(hardwareMap, "front_right_drive"),
                                 new MotorEx(hardwareMap, "back_left_drive"),
                                 new MotorEx(hardwareMap, "back_right_drive"));
        time = new ElapsedTime();
        time.reset();

        gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        headingPID = new HeadingPID(telemetry, gyro);
    }

    @Override
    public void loop() {
        headingPID.setTurnConstant(turnConstant);
        double timePassed = time.seconds();
        double headingCorrection = headingPID.runPID(timePassed, previousTurning);
        double[] driveCoefficients = {
                gamepad.getLeftX(),
                gamepad.getLeftY(),
                Math.abs(gamepad.getRightX()) > 0.1 ? gamepad.getRightX() : headingCorrection
        };
        for (int i = 0; i < driveCoefficients.length; i++) {
            driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
            driveCoefficients[i] = Range.clip(driveCoefficients[i], -1, 1 ); // clip in case of multiplier that is greater than 1
        }
        drive.driveRobotCentric(driveCoefficients[0], driveCoefficients[1], driveCoefficients[2]);
        previousTurning = gamepad.getRightX();
        time.reset();
//        drive.driveRobotCentric(0,0,0.3);

        telemetry.addData("Previous turn input: ", previousTurning);
        telemetry.addData("Time passed: ", timePassed);
        telemetry.addData("Heading correction: ", headingCorrection);
        telemetry.addData("Forward fed to dt: ", driveCoefficients[1]);
        telemetry.addData("Strafing fed to dt: ", driveCoefficients[0]);
        telemetry.addData("Turning fed to dt: ", driveCoefficients[2]);
        telemetry.update();
    }
}
