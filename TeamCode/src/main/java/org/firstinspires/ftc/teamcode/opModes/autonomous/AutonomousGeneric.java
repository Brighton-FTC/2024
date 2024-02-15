package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.util.Pair;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.util.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.jetbrains.annotations.Contract;

/**
 * Generic autonomous.
 * <br />
 * To use, extend and overwrite <code>posesContainer</code> and <code>goesToPixelStackFirst</code> with the necessary values,
 * and add annotate the class with <code>@Autonomous</code>.
 * <br />
 * TODO: this code is written using the grabber, but once active intake code is written, replace with that.
 */
public abstract class AutonomousGeneric extends LinearOpMode {
    // TODO: replace with custom tfod values if needed
    public static final String TFOD_PROCESSOR_NAME = "CenterStage.tflite";
    public static final String[] TFOD_LABELS = {"Pixel"};

    public static final Size CAMERA_RESOLUTION = new Size(640, 480);

    // overwrite in subclasses
    protected PosesContainer posesContainer;
    protected boolean goesToPixelStackFirst;

    private TfodProcessor tfod;

    private SampleMecanumDrive drive;

    private ArmComponent arm;
    private LinearSlideComponent linearSlide;
    private GrabberComponent grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);

        arm = new ArmComponent(new MotorEx(hardwareMap, "armMotor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linearSlideMotor"), arm);
        grabber = new GrabberComponent(
                new SimpleServo(hardwareMap, "grabberServo1", 0, 360),
                new SimpleServo(hardwareMap, "grabberServo2", 0, 360)
        );

        // initialize vision
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(tfod)
                .setCameraResolution(CAMERA_RESOLUTION)
                .build();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_PROCESSOR_NAME)
                .setModelLabels(TFOD_LABELS)
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

        grabber.open();
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

        grabber.close(); // TODO: find more accurate way of picking up pixel
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

        grabber.open(); // TODO: find more accurate way of placing pixel
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
        Recognition recognition = tfod.getRecognitions().get(0);
        int recognitionX = (int) (recognition.getLeft() + recognition.getRight()) / 2;

        // check which third of the screen the object is in
        if (recognitionX < CAMERA_RESOLUTION.getWidth() / 3) {
            return new Pair<>(posesContainer.leftSpikeMarkPose, posesContainer.leftBackdropPose);

        } else if (recognitionX > CAMERA_RESOLUTION.getWidth() * 2 / 3) {
            return new Pair<>(posesContainer.rightSpikeMarkPose, posesContainer.rightBackdropPose);

        } else {
            return new Pair<>(posesContainer.centerSpikeMarkPose, posesContainer.centerBackdropPose);
        }
    }
}
