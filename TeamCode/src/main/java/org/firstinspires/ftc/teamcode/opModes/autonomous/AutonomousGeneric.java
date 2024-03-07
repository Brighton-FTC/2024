package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.util.Pair;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.components.vision.eocv.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.components.vision.eocv.ColourMassDetectionProcessor.PropPositions;
import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;
import org.firstinspires.ftc.teamcode.util.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.Contract;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Generic autonomous.
 * <br />
 * To use, extend and overwrite <code>posesContainer</code> and <code>goesToPixelStackFirst</code> with the necessary values,
 * and add annotate the class with <code>@Autonomous</code>.
 * <br />
 */

@Autonomous(name = "Autonomous Generic", group = "autonomous-test")
public class AutonomousGeneric extends LinearOpMode {
    // TODO: replace with custom tfod values if needed
    public static final Size CAMERA_RESOLUTION = new Size(640, 480);

    public static final Scalar LOWER_DETECTION_BOUND = new Scalar(150, 100, 100);
    public static final Scalar UPPER_DETECTION_BOUND = new Scalar(180, 255, 255);
    public static final double MIN_DETECTION_AREA = 100;

    public static final double STARTING_POSE_ERROR = 0.2;

    // overwrite in subclasses
    protected PosesContainer posesContainer;
    protected boolean goesToPixelStackFirst;

    private ColourMassDetectionProcessor colorMassDetectionProcessor;
    private AprilTagProcessor aprilTag;

    private SampleMecanumDrive drive;

    private ArmComponent arm;
    private LinearSlideComponent linearSlide;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    // these are filled in already, they're distance from camera to center of bot
    public static final double cameraCenterXOffsetInches = 8.5;
    public static final double cameraCenterYOffsetInches = 0;
    public static final Vector2d deltaF = new Vector2d(8.5, 0);

    private final BCGyro gyro = new BCGyro(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        posesContainer = getPosesContainer();

        // initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);

        arm = new ArmComponent(new MotorEx(hardwareMap, "armMotor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linearSlideMotor"), arm);
        activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "activeIntakeMotor"));
        outtake = new OuttakeComponent(new SimpleServo(hardwareMap, "outtakeServo", 0, 360));

        // initialize vision
        colorMassDetectionProcessor = new ColourMassDetectionProcessor(
                LOWER_DETECTION_BOUND,
                UPPER_DETECTION_BOUND,
                () -> MIN_DETECTION_AREA,
                () -> CAMERA_RESOLUTION.getWidth() / 3.0,
                () -> CAMERA_RESOLUTION.getWidth() * 2.0 / 3.0
        );

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessors(colorMassDetectionProcessor, aprilTag)
                .setCameraResolution(CAMERA_RESOLUTION)
                .build();

        waitForStart();

        Pair<Pose2d, Pose2d> correctPoses = getCorrectPoses();

        placePurplePixel(correctPoses.first);

        if (goesToPixelStackFirst) {
            pickUpPixel(posesContainer.pixelStackPose);
        }

        while (opModeIsActive()) {
            placePixel(correctPoses.second);
            pickUpPixel(posesContainer.pixelStackPose);
        }

        visionPortal.close();
    }

    /**
     * Determine the starting point and get the corresponding {@link PosesContainer} object.
     *
     * @return The corresponding poses container.
     */
    private PosesContainer getPosesContainer() {
        // TODO: fill in positions
        Vector2d redAudience = new Vector2d();
        Vector2d redFarSide = new Vector2d();
        Vector2d blueAudience = new Vector2d();
        Vector2d blueFarSide = new Vector2d();

        Vector2d currentPosition = drive.getPoseEstimate().vec();

        if (currentPosition.distTo(redAudience) <= STARTING_POSE_ERROR) {
            return PosesContainer.RED_AUDIENCE_POSES;

        } else if (currentPosition.distTo(redFarSide) <= STARTING_POSE_ERROR) {
            return PosesContainer.RED_FAR_SIDE_POSES;

        } else if (currentPosition.distTo(blueAudience) <= STARTING_POSE_ERROR) {
            return PosesContainer.BLUE_AUDIENCE_POSES;

        } else if (currentPosition.distTo(blueFarSide) <= STARTING_POSE_ERROR) {
            return PosesContainer.BLUE_FAR_SIDE_POSES;

        } else {
            return null;
        }
    }

    /**
     * Get the robot to place a purple pixel on the correct spike mark
     *
     * @param endPose The pose where the robot needs to be to place the pixel.
     */
    private void placePurplePixel(Pose2d endPose) {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            arm.lower();
                            linearSlide.lower();

                            while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                arm.moveToSetPoint();
                                linearSlide.moveToSetPoint();
                            }
                        })
                        .lineToLinearHeading(endPose)
                        .build()
        );
        setPoseAtag();
        outtake.releasePixel();
    }

    /**
     * Pick up a pixel from a pixel stack.
     *
     * @param endPose The pose where the robot needs to be to pick up the pixel.
     */
    private void pickUpPixel(Pose2d endPose) {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            arm.lift();
                            linearSlide.lower();

                            while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                arm.moveToSetPoint();
                                linearSlide.moveToSetPoint();
                            }
                        })
                        .lineToLinearHeading(endPose)
                        .build()
        );
        setPoseAtag();
        activeIntake.turnManually();
    }

    /**
     * Place a pixel on the backdrop.
     *
     * @param endPose The pose where the robot needs to be to place the pixel.
     */
    private void placePixel(Pose2d endPose) {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addTemporalMarker(0, () -> {
                            arm.lift();
                            linearSlide.lift();

                            while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                arm.moveToSetPoint();
                                linearSlide.moveToSetPoint();
                            }
                        })
                        .lineToLinearHeading(endPose)
                        .build()
        );
        setPoseAtag();
        activeIntake.turnManually();
    }

    /**
     * Park the robot in the backstage area.
     *
     * @param endPose The place for the robot to be parked.
     */
    private void park(Pose2d endPose) {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(endPose)
                        .build()
        );
    }

    /**
     * Determine which spike mark the white pixel/team prop is on.
     *
     * @return The poses for the correct spike mark, and the correct backdrop side.
     */
    @NonNull
    @Contract(" -> new")
    private Pair<Pose2d, Pose2d> getCorrectPoses() {
        PropPositions propPosition = colorMassDetectionProcessor.getRecordedPropPosition();

        // check which third of the screen the object is in
        if (propPosition == PropPositions.LEFT) {
            return new Pair<>(posesContainer.leftSpikeMarkPose, posesContainer.leftBackdropPose);

        } else if (propPosition == PropPositions.RIGHT) {
            return new Pair<>(posesContainer.rightSpikeMarkPose, posesContainer.rightBackdropPose);

        } else { // middle or not found
            return new Pair<>(posesContainer.centerSpikeMarkPose, posesContainer.centerBackdropPose);
        }
    }

    @Nullable
    private Pose2d getPoseFromAtag(){
        // get the detections and filter out the ones without metadata
        List<AprilTagDetection> detections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .collect(Collectors.toList());

        if (detections == null || detections.isEmpty()) {
            return null;
        }
        AprilTagDetection OurTag = detections.get(0);
        for (AprilTagDetection d : detections) {
            if (OurTag.ftcPose == null) {
                OurTag = d;
                continue;
            }
            if (d.ftcPose == null) { continue; }
            if (Math.abs(d.ftcPose.x) < Math.abs(OurTag.ftcPose.x)) {
                OurTag = d; }
        }
        if (OurTag.ftcPose == null) {
            return null;
        }

        Vector2d newDeltaF= deltaF.rotated(-OurTag.ftcPose.yaw);
        Vector2d cameraVector = new Vector2d(OurTag.ftcPose.y, -OurTag.ftcPose.x).rotated(-OurTag.ftcPose.yaw);

        VectorF tagPosition = OurTag.metadata.fieldPosition;
        Vector2d rTag = new Vector2d(tagPosition.get(0),tagPosition.get(1));

        Vector2d returnVector = rTag.minus(newDeltaF);
        returnVector = returnVector.minus(cameraVector);

        Pose2d returnPose = new Pose2d(returnVector,
                Math.toRadians(-OurTag.ftcPose.yaw));

        return returnPose;
    }

    private void setPoseAtag(){
        Pose2d pose = getPoseFromAtag();
        if (pose != null){
            drive.setPoseEstimate(pose);
        }
    }
}
