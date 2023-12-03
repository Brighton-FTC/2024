package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test for Rotation Speed", group = "speed-test")
public class rotateSpeedTest extends OpMode {

    private MecanumDrive mecanumDrive;
    private MecanumDriveOdometry mecanumDriveOdometry;
    private IMU imu;
    private ElapsedTime time;
    private Motor[] mecanumMotors;

    // TODO: Tune in METERS
    public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5)
    );

    @Override
    public void init() {
        mecanumMotors = new Motor[]{
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        mecanumDrive = new MecanumDrive(mecanumMotors[0], mecanumMotors[1], mecanumMotors[2], mecanumMotors[3]);
        time = new ElapsedTime();
        time.startTime();
        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Determine if logo faces left or right
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        mecanumDriveOdometry = new MecanumDriveOdometry(MECANUM_KINEMATICS, new Rotation2d(), new Pose2d());
    }

    @Override
    public void loop() {
        // drive mecanum for 5 seconds
        if (time.time(TimeUnit.SECONDS) < 5) {
            mecanumDrive.driveRobotCentric(0, 0, 1);
        }

        // updates odometry
        mecanumDriveOdometry.updateWithTime(time.time(TimeUnit.SECONDS),
                Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)),
                new MecanumDriveWheelSpeeds(
                        mecanumMotors[0].getRate(),
                        mecanumMotors[1].getRate(),
                        mecanumMotors[2].getRate(),
                        mecanumMotors[3].getRate()
                ));

        // gets current from each hub and prints in telemetry
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            double current = module.getCurrent(CurrentUnit.AMPS);
            String device_name = String.format("Device %s current is: ", module.getDeviceName());

            telemetry.addData(device_name, current);
        }

        telemetry.addData("Odometry position: ", mecanumDriveOdometry.getPoseMeters().getHeading());
        telemetry.update();
    }
}
