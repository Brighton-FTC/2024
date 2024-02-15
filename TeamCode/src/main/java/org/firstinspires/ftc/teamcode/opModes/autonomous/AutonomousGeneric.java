package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.util.Size;

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

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ArmComponent arm = new ArmComponent(new MotorEx(hardwareMap, "armMotor"));
        LinearSlideComponent linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linearSlideMotor"), arm);
        GrabberComponent grabber = new GrabberComponent(
                new SimpleServo(hardwareMap, "grabberServo1", 0, 360),
                new SimpleServo(hardwareMap, "grabberServo2", 0, 360)
        );

        waitForStart();

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

        // go to spike marks and place the purple pixel.
        Pose2d correctSpikeMarkPose = getCorrectSpikeMarkPose();

        drive.followTrajectory(
                drive.trajectoryBuilder(posesContainer.getStartingPose())
                        .addTemporalMarker(0, () -> {
                            arm.lower();
                            linearSlide.lower();

                            while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                arm.moveToSetPoint();
                                linearSlide.moveToSetPoint();
                            }
                        })
                        .lineToLinearHeading(correctSpikeMarkPose)
                        .build()
        );

        grabber.open();

        if (goesToPixelStackFirst) {
            // go from spike marks to pixel stack and pick up white pixel
            drive.followTrajectory(
                    drive.trajectoryBuilder(correctSpikeMarkPose)
                            .addTemporalMarker(0, () -> {
                                arm.lift();
                                linearSlide.lower();

                                while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                    arm.moveToSetPoint();
                                    linearSlide.moveToSetPoint();
                                }
                            })
                            .lineToLinearHeading(posesContainer.getPixelStackPose())
                            .build()
            );

            grabber.close(); // TODO: find more accurate way of picking up pixel
        }

        // go to backdrop and place yellow (and purple if goesToPixelStackFirst is true) pixels,
        // then go and pick up another pixel, repeat
        Pose2d correctBackdropPose;

        if (correctSpikeMarkPose.equals(posesContainer.getLeftSpikeMarkPose())) {
            correctBackdropPose = posesContainer.getLeftBackdropPose();

        } else if (correctSpikeMarkPose.equals(posesContainer.getRightBackdropPose())) {
            correctBackdropPose = posesContainer.getRightBackdropPose();

        } else {
            correctBackdropPose = posesContainer.getCenterBackdropPose();
        }

        while (opModeIsActive()) {
            // go to backdrop
            drive.followTrajectory(
                    drive.trajectoryBuilder(goesToPixelStackFirst ? posesContainer.getPixelStackPose() : correctSpikeMarkPose)
                            .addTemporalMarker(0, () -> {
                                arm.lift();
                                linearSlide.lift();

                                while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                    arm.moveToSetPoint();
                                    linearSlide.moveToSetPoint();
                                }
                            })
                            .lineToLinearHeading(correctBackdropPose)
                            .build()
            );

            grabber.open(); // TODO: find more accurate way of placing pixel

            // go from spike marks to pixel stack and pick up white pixel
            drive.followTrajectory(
                    drive.trajectoryBuilder(correctSpikeMarkPose)
                            .addTemporalMarker(0, () -> {
                                arm.lift();
                                linearSlide.lower();

                                while (arm.atSetPoint() && linearSlide.atSetPoint()) {
                                    arm.moveToSetPoint();
                                    linearSlide.moveToSetPoint();
                                }
                            })
                            .lineToLinearHeading(posesContainer.getPixelStackPose())
                            .build()
            );

            grabber.close(); // TODO: find more accurate way of picking up pixel
        }

        visionPortal.close();
    }

    /**
     * Determine which spike mark the white pixel/team prop is on.
     *
     * @return The pose of that spike mark.
     */
    private Pose2d getCorrectSpikeMarkPose() {
        Recognition recognition = tfod.getRecognitions().get(0);
        int recognitionX = (int) (recognition.getLeft() + recognition.getRight()) / 2;

        // check which third of the screen the object is in
        if (recognitionX < CAMERA_RESOLUTION.getWidth() / 3) {
            return posesContainer.getLeftSpikeMarkPose();

        } else if (recognitionX > CAMERA_RESOLUTION.getWidth() * 2 / 3) {
            return posesContainer.getRightSpikeMarkPose();

        } else {
            return posesContainer.getCenterSpikeMarkPose();
        }
    }
}
