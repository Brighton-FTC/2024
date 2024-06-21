package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.example.meepmeeptesting.trajectories.Drive;
import com.example.meepmeeptesting.trajectories.PosesContainer;
import com.example.meepmeeptesting.trajectories.TrajectoriesFactory;
import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.RandomizationState;
import com.example.meepmeeptesting.util.StartingSide;
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

public abstract class AutonomousGeneric extends LinearOpMode {
    // TODO: replace with custom tfod values if needed
    public static final Size CAMERA_RESOLUTION = new Size(640, 480);
    public static final double MIN_DETECTION_AREA = 100;
    private ColourMassDetectionProcessor colorMassDetectionProcessor;
    private AprilTagProcessor aprilTag;

    private MecanumDrive drive;

    private ArmComponent arm;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    private TrajectoriesFactory trajectoriesFactory;

    // these are filled in already, they're distance from camera to center of bot
    public static final Vector2d DELTA_F = new Vector2d(8.5, 0);

    private final AllianceColor alliance;
    private final StartingSide startingSide;
    private RandomizationState randomization;
    //		// RED NEW 3DP NATIONALS
    // tight, doesn't work if dark, but safe
//    private static final Scalar redLower = new Scalar(100, 130, 0);
//    private static final Scalar redUpper = new Scalar(115, 220, 130);

    // safe one
    static final Scalar redLower = new Scalar(100, 100, 0);
    static final Scalar redUpper = new Scalar(120, 220, 120);

    // BLUE NEW 3DP NATIONALS
    static final Scalar blueLower = new Scalar(170, 210, 50) ;
    static final Scalar blueUpper = new Scalar(190, 245, 180) ;

    protected AutonomousGeneric(AllianceColor alliance, StartingSide startingSide) {
        super();

        this.alliance = alliance;
        this.startingSide = startingSide;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "active_intake_motor_left"));
        outtake = new OuttakeComponent(new SimpleServo(hardwareMap, "outtake_front_servo", 0, 360), new SimpleServo(hardwareMap, "outtake_back_servo", 0, 360));

        Scalar LOWER_DETECTION_BOUND = alliance == AllianceColor.BLUE ? blueLower : redLower;
        Scalar UPPER_DETECTION_BOUND = alliance == AllianceColor.BLUE ? blueUpper : redUpper;

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
        randomization = getRandomization();

        trajectoriesFactory = new TrajectoriesFactory(new MecanumDriveAdaptor(drive), alliance, startingSide, randomization);

        Action driveToCorrectSpikeMarkAction = trajectoriesFactory.startToSpike();

        Action placePixelOnGroundAction = new SequentialAction(
                arm.goToStateAction(ArmComponent.State.PLACE_GROUND),
                outtake.releaseBackAction()
        );

        Action driveToBackdropFromSpikeMarksAction;
        if (startingSide ==  StartingSide.AUDIENCE_SIDE) {
            driveToBackdropFromSpikeMarksAction = trajectoriesFactory.audienceSpikeToBackdrop();
        } else {
            driveToBackdropFromSpikeMarksAction = trajectoriesFactory.farSpikeToBackdrop();
        }

        Action placePixelsOnBackdropAction = new SequentialAction(
                arm.goToStateAction(ArmComponent.State.PLACE_LOW_BACKDROP),
                outtake.releaseFrontAction()
        );

        Action parkAction = trajectoriesFactory.park();

        waitForStart();

        Actions.runBlocking(driveToCorrectSpikeMarkAction);
        Actions.runBlocking(placePixelOnGroundAction);

        Actions.runBlocking(driveToBackdropFromSpikeMarksAction);
        Actions.runBlocking(placePixelsOnBackdropAction);

        Actions.runBlocking(parkAction);

        visionPortal.close();
    }

    @NonNull
    private RandomizationState getRandomization() {
        PropPositions propPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        return propPosition.getCorrespondingRandomization();
    }

    private static class MecanumDriveAdaptor implements Drive {
        private final MecanumDrive drive;

        public MecanumDriveAdaptor(MecanumDrive drive) {
            this.drive = drive;
        }

        @Override
        public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
            return drive.actionBuilder(beginPose);
        }
    }
}