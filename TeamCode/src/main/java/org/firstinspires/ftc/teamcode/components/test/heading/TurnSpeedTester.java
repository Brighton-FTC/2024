package org.firstinspires.ftc.teamcode.components.test.heading;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
@Autonomous(name = "Turn Speed Tester", group = "tuner")
public class TurnSpeedTester extends OpMode {

    private IMU imu;
    private BCGyro gyro;

    private MecanumDrive drive;

    private ElapsedTime time;

    @Override
    public void init() {
        gyro = new BCGyro(hardwareMap);
        gyro.init(
                new BNO055IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        drive = new MecanumDrive(new MotorEx(hardwareMap, "front_left_drive"),
                                 new MotorEx(hardwareMap, "front_right_drive"),
                                 new MotorEx(hardwareMap, "back_left_drive"),
                                 new MotorEx(hardwareMap, "back_right_drive"));
        time = new ElapsedTime();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

    }

    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        if (time.milliseconds() < 5000) {
            drive.driveRobotCentric(0, 0, 1);

            telemetry.addLine("Turning");
        }
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Now use these simple methods to extract each angle
        // (Java type double) from the object you just created:
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("BNOIMU: ", yaw);
        telemetry.addData("BCGyro: ", gyro.getHeading());
        telemetry.addData("Time: ", time.milliseconds());
        telemetry.update();
    }
}
