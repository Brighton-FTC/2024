package org.firstinspires.ftc.teamcode.basicLocalizationStack.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

/**
 * A very basic attempt at a localization stack (<b>untested and in development</b>). <br />
 * <p>
 * The code uses a webcam, odometry, and in the future will use four distance sensors. <br />
 *
 * Most measurements are in meters.
 */
public class BasicLocalizationStack {
    public static final double FIELD_SIZE = DistanceUnit.INCH.toMeters(72);

    public static final double ROBOT_LENGTH = 0.5;
    public static final double ROBOT_WIDTH = 0.5;

    private final double headingOffset;

    private final SensorDistanceEx frontSensor, backSensor, leftSensor, rightSensor;

    private final GyroEx gyro;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    private final HashMap<Integer, Point> nearSideAprilTags = new HashMap<>(); // stores the ids and positions of the april tags on the field. (distances are in inches.)
    private final HashMap<Integer, Point> farSideAprilTags = new HashMap<>(); // stores the ids and positions of the april tags on the field. (distances are in inches.)

    /**
     * A very basic attempt at a localization stack.
     *
     * @param hardwareMap The hardware map.
     * @param headingOffset The offset from the robot's heading.
     */
    public BasicLocalizationStack(HardwareMap hardwareMap, double headingOffset) {
        this.headingOffset = headingOffset < 0 ? headingOffset + 360 : headingOffset;

        nearSideAprilTags.put(10, new Point(18, 72));
        nearSideAprilTags.put(7, new Point(54, 72));

        farSideAprilTags.put(1, new Point(15, 6));
        farSideAprilTags.put(2, new Point(18, 6));
        farSideAprilTags.put(3, new Point(21, 6));
        farSideAprilTags.put(4, new Point(51, 6));
        farSideAprilTags.put(5, new Point(54, 6));
        farSideAprilTags.put(6, new Point(57, 6));

        frontSensor = new SensorRevTOFDistance(hardwareMap, "front_sensor");
        backSensor = new SensorRevTOFDistance(hardwareMap, "back_sensor");
        rightSensor = new SensorRevTOFDistance(hardwareMap, "right_sensor");
        leftSensor = new SensorRevTOFDistance(hardwareMap, "left_sensor");

        gyro = new RevIMU(hardwareMap, "gyro");

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
    }

    /**
     * A very basic attempt at a localization stack.
     *
     * @param hardwareMap The hardware map.
     */
    public BasicLocalizationStack(HardwareMap hardwareMap) {
        this(hardwareMap, 0);
    }

    /**
     * Get the approximate location of the robot (in inches), calculated from april tag positions, relative to the top left corner of the pitch.
     *
     * @return A {@link Pose2d} object representing the position and rotation of the robot.
     */
    @Nullable
    public Pose2d getAprilTagLocationEstimate() {
        List<AprilTagDetection> aprilTagDetections = getAprilTagDetections().stream()
                .filter((detection) -> nearSideAprilTags.containsKey(detection.id) || farSideAprilTags.containsKey(detection.id))
                .collect(Collectors.toList());

        if (aprilTagDetections.size() == 0) {
            return null;

        } else {
            double[] x = new double[aprilTagDetections.size()];
            double[] y = new double[aprilTagDetections.size()];

            int i = 0;
            for (AprilTagDetection detection : aprilTagDetections) {
                if (nearSideAprilTags.containsKey(detection.id)) {
                    x[i] = Objects.requireNonNull(nearSideAprilTags.get(detection.id)).x + detection.ftcPose.x;
                    y[i] = Objects.requireNonNull(nearSideAprilTags.get(detection.id)).y - detection.ftcPose.y + ROBOT_LENGTH / 2;

                } else if (farSideAprilTags.containsKey(detection.id)) {
                    x[i] = Objects.requireNonNull(farSideAprilTags.get(detection.id)).x + detection.ftcPose.x;
                    y[i] = Objects.requireNonNull(farSideAprilTags.get(detection.id)).y + detection.ftcPose.y + ROBOT_LENGTH / 2;
                }

                i++;
            }

            return new Pose2d(
                    new Translation2d(Arrays.stream(x)
                            .map(DistanceUnit.INCH::toMeters)
                            .average()
                            .orElse(0),

                            Arrays.stream(y)
                                    .map(DistanceUnit.INCH::toMeters)
                                    .average()
                                    .orElse(0)),

                    Rotation2d.fromDegrees(gyro.getHeading() - headingOffset)
            );
        }
    }

    /**
     * Gets a MecanumDriveOdometry object given the necessary info.
     * @param wheelPositions The positions of the wheels, relative to the center of the robot, in meters.
     * @param robotPose The position of the robot, relative to the origin (wherever you decide (0, 0) to be), in meters
     * @return A {@link MecanumDriveOdometry} object which corresponds to the robot.
     */
    public MecanumDriveOdometry getOdometryObject(@NonNull Translation2d[] wheelPositions, @NonNull Pose2d robotPose) {
        if (wheelPositions.length != 4) {
            throw new IllegalArgumentException("Array 'wheelPositions' must have length 4, but was length " + wheelPositions.length);
        }

        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                wheelPositions[0],
                wheelPositions[1],
                wheelPositions[2],
                wheelPositions[3]
        );

        return new MecanumDriveOdometry(
                kinematics, Rotation2d.fromDegrees(gyro.getHeading() - headingOffset), robotPose
        );
    }

    /**
     * Get an estimate of the robot's position, given motor encoders.
     * @param odometry The mecanum odometry that represents the robot.
     * @param encoders The motor encoders of the wheels. Should be ordered {frontLeft, frontRight, backLeft, backRight}.
     * @param currentTime The time (in seconds) since the start of the opmode.
     * @return A {@link Pose2d} object representing the position and rotation of the robot.
     */
    public Pose2d getOdometryLocationEstimate(@NonNull MecanumDriveOdometry odometry, @NonNull Motor.Encoder[] encoders, double currentTime) {
        if (encoders.length != 4) {
            throw new IllegalArgumentException("Array 'encoders' must have length 4, but was length " + encoders.length);
        }

        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                encoders[0].getRate(), encoders[1].getRate(),
                encoders[2].getRate(), encoders[3].getRate()
        );

        return odometry.updateWithTime(
                currentTime,
                Rotation2d.fromDegrees(gyro.getHeading() - headingOffset),
                wheelSpeeds
        );
    }

    /**
     * Get an estimate of the robot's position, given motor encoders.
     * @param odometry The mecanum odometry that represents the robot.
     * @param motors The wheel motors. Should be ordered {frontLeft, frontRight, backLeft, backRight}.
     * @param currentTime The time (in seconds) since the start of the opmode.
     * @return A {@link Pose2d} object representing the position and rotation of the robot.
     */
    public Pose2d getOdometryLocationEstimate(@NonNull MecanumDriveOdometry odometry, @NonNull Motor[] motors, double currentTime) {
        if (motors.length != 4) {
            throw new IllegalArgumentException("Array 'encoders' must have length 4, but was length " + motors.length);
        }

        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                motors[0].getRate(), motors[1].getRate(),
                motors[2].getRate(), motors[3].getRate()
        );

        return odometry.updateWithTime(
                currentTime,
                Rotation2d.fromDegrees(gyro.getHeading() - headingOffset),
                wheelSpeeds
        );
    }

    private List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections().stream()
                .filter((detection) -> detection.metadata != null)
                .collect(Collectors.toList());
    }
}
