package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.util.Pair;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.components.vision.eocv.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.components.vision.eocv.ColourMassDetectionProcessor.PropPositions;
import org.firstinspires.ftc.teamcode.util.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

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

    private ColourMassDetectionProcessor colorMassDetectionProcessor;
    private AprilTagProcessor aprilTag;

    private MecanumDrive drive;

    private ArmComponent arm;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    // TODO: define these actions
    private Action driveToCorrectSpikeMarkAction,
            placePixelOnGroundAction,
            placePixelsOnBackdropAction,
            intakePixelsAction,
            driveToBackdropFromSpikeMarksAction,
            driveToBackdropFromPixelStackAction,
            driveToPixekStackAction;

    // these are filled in already, they're distance from camera to center of bot
    public static final Vector2d DELTA_F = new Vector2d(8.5, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"), new SimpleServo(hardwareMap, "outtake_rotation_servo", 0, 360));
        activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "active_intake_motor_left"), new MotorEx(hardwareMap, "active_intake_motor_rigth"));
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

        // initialize actions
        posesContainer = getPosesContainer();
        Pair<Pose2d, Pose2d> correctPoses = getCorrectPoses();

        driveToCorrectSpikeMarkAction = drive.actionBuilder(drive.pose)
                .splineTo(correctPoses.first.position, correctPoses.first.heading)
                .build();

        placePixelOnGroundAction = new SequentialAction(
                arm.goToStateAction(ArmComponent.State.PLACE_GROUND),
                outtake.releasePixelAction()
        );

        placePixelsOnBackdropAction = new SequentialAction(
                arm.goToStateAction(ArmComponent.State.HIGH),
                outtake.releaseAllPixelsAction()
        );

        intakePixelsAction = new SequentialAction(
                activeIntake.turnManuallyAction(),
                activeIntake.turnManuallyAction()
        );

        driveToBackdropFromSpikeMarksAction = drive.actionBuilder(correctPoses.first)
                .splineTo(correctPoses.second.position, correctPoses.second.heading)
                .build();

        driveToBackdropFromPixelStackAction = drive.actionBuilder(posesContainer.pixelStackPose)
                .splineTo(posesContainer.centerBackdropPose.position, posesContainer.centerBackdropPose.heading)
                .build();

        waitForStart();

        Actions.runBlocking(driveToCorrectSpikeMarkAction);
        Actions.runBlocking(placePixelOnGroundAction);
        Actions.runBlocking((drive.actionBuilder(drive.pose)
                .splineTo(posesContainer.centerSpikeMarkPose.position, posesContainer.centerSpikeMarkPose.heading)
                .build())); // used to standardise robot position after spike mark (as robot could be in 3 different positions.
        Actions.runBlocking(driveToBackdropFromSpikeMarksAction);
        Actions.runBlocking(placePixelsOnBackdropAction);

        while (opModeIsActive()) {
            Actions.runBlocking(driveToPixekStackAction);
            Actions.runBlocking(intakePixelsAction);
            Actions.runBlocking(driveToBackdropFromPixelStackAction);
            Actions.runBlocking(placePixelsOnBackdropAction);
        }

        visionPortal.close();
    }

    private double distance(Vector2d u, Vector2d v) {
        return Math.hypot(u.x - v.x, u.y - v.y);
    }

    /**
     * Determine the starting point and get the corresponding {@link PosesContainer} object.
     *
     * @return The corresponding poses container.
     */
    @Nullable
    private PosesContainer getPosesContainer() {
        // TODO: fill in positions
        Vector2d redAudience = PosesContainer.RED_AUDIENCE_POSES.startingPose.position;
        Vector2d redFarSide = PosesContainer.RED_FAR_SIDE_POSES.startingPose.position;
        Vector2d blueAudience = PosesContainer.BLUE_AUDIENCE_POSES.startingPose.position;
        Vector2d blueFarSide = PosesContainer.BLUE_FAR_SIDE_POSES.startingPose.position;

        Vector2d currentPosition = drive.pose.position;

        if (distance(currentPosition, redAudience) <= STARTING_POSE_ERROR) {
            return PosesContainer.RED_AUDIENCE_POSES;

        } else if (distance(currentPosition, redFarSide) <= STARTING_POSE_ERROR) {
            return PosesContainer.RED_FAR_SIDE_POSES;

        } else if (distance(currentPosition, blueAudience) <= STARTING_POSE_ERROR) {
            return PosesContainer.BLUE_AUDIENCE_POSES;

        } else if (distance(currentPosition, blueFarSide) <= STARTING_POSE_ERROR) {
            return PosesContainer.BLUE_FAR_SIDE_POSES;

        } else {
            return null;
        }
    }

    @NonNull
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
}