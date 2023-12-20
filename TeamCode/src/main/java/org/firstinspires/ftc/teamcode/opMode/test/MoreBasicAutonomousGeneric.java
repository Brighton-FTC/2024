package org.firstinspires.ftc.teamcode.opMode.test;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Don't use this for now, there are changes not applied here - go to Steve and get this fixed first
 */
@Disabled
@Autonomous(name = "More basic autonomous (test first)", group = "autonomous-test")
public class MoreBasicAutonomousGeneric extends LinearOpMode {
    // TODO: once the custom model exists, replace this with the custom model name
    public static final String WHITE_PIXELS_MODEL_ASSET = "CenterStage.tflite";
    public static final String TEAM_PROP_MODEL_ASSET = "TeamPropModel_v1.0.tflite";

    // TODO: once the custom model exists, modify this
    public static final String[] WHITE_PIXEL_LABELS = {"Pixel"};
    public static final String[] TEAM_PROP_LABELS = {"red_pixel", "blue_pixel"};

    public String[] WANTED_LABELS = {
            "Pixel",
            "red_pixel",
            "blue_pixel"
    };


    protected VisionPortal visionPortal;

    protected TfodProcessor[] tfodModels;
    protected AprilTagProcessor aprilTag;

    public static final Size CAMERA_SIZE = new Size(640, 480);

    public static final int TFOD_RECOGNITION_SCREEN_POS = (int) (CAMERA_SIZE.getHeight() * 4.0 / 5.0);

    protected Motor[] mecanumMotors;
    protected MecanumDrive mecanum;

    protected ArmComponent arm;
    protected LinearSlideComponent linearSlide;
    protected GrabberComponent grabber;

    protected GyroEx gyro;


    protected MecanumDriveOdometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware

        // store all the motors separately, for odometry
        mecanumMotors = new Motor[]{
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };


        mecanum = new MecanumDrive(mecanumMotors[0], mecanumMotors[1], mecanumMotors[2], mecanumMotors[3]);

        for (Motor motor : mecanumMotors) {
            motor.setInverted(!motor.getInverted());
        }

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linear_slide_motor"));
        grabber = new GrabberComponent(new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360),
                new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360));

        // vision stuff
        aprilTag = new AprilTagProcessor.Builder().build();

        // TODO: find out what the input size and aspect ratio things are and if we need them
        tfodModels = new TfodProcessor[]{
                new TfodProcessor.Builder()
                        .setModelAssetName(WHITE_PIXELS_MODEL_ASSET)
                        .setModelLabels(WHITE_PIXEL_LABELS)
                        .setIsModelTensorFlow2(true)
                        .setIsModelQuantized(true)
//                        .setModelInputSize(0)
//                        .setModelAspectRatio(0)
                        .build(),

                new TfodProcessor.Builder()
                        .setModelAssetName(TEAM_PROP_MODEL_ASSET)
                        .setModelLabels(TEAM_PROP_LABELS)
                        .setIsModelTensorFlow2(true)
                        .setIsModelQuantized(true)
//                        .setModelInputSize(0)
//                        .setModelAspectRatio(0)
                        .build()
        };

        for (TfodProcessor model : tfodModels) {
            model.setMinResultConfidence(0.75F); // TODO: set
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .setCameraResolution(CAMERA_SIZE)
                .enableLiveView(true)
                .addProcessors(tfodModels)
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        // movement

        PixelSide pixelSide;
        Recognition recognition;

        while (true) {
            List<Recognition> tfodRecognitions = getTfodDetections();

            if (tfodRecognitions.size() == 0) {
                mecanum.driveRobotCentric(0, 0.5, 0);
            } else {
                recognition = tfodRecognitions.stream()
                        .sorted(Comparator.comparingDouble(Recognition::getBottom))
                        .collect(Collectors.toList()).get(0);

                boolean isDone = driveToTfodRecognition(recognition);

                if (isDone) {
                    int recognitionX = (int) ((recognition.getLeft() + recognition.getRight()) / 2);

                    if (recognitionX < CAMERA_SIZE.getWidth() / 3) {
                        pixelSide = PixelSide.LEFT;

                    } else if (recognitionX > CAMERA_SIZE.getWidth() * 2.0 / 3.0) {
                        pixelSide = PixelSide.RIGHT;

                    } else {
                        pixelSide = PixelSide.CENTER;
                    }

                    break;
                }
            }

            sleep(20);
        }

        int recognitionX;

        if (pixelSide == PixelSide.LEFT) {
            do {
                mecanum.driveRobotCentric(0, 0, -0.5);

                recognitionX = (int) ((recognition.getLeft() + recognition.getRight()) / 2);

                sleep(20);

            } while (2.0 / 5.0 < recognitionX && recognitionX < 3.0 / 5.0);

        } else if (pixelSide == PixelSide.RIGHT) {
            do {
                mecanum.driveRobotCentric(0, 0, 0.5);

                recognitionX = (int) ((recognition.getLeft() + recognition.getRight()) / 2);

                sleep(20);

            } while (2.0 / 5.0 < recognitionX && recognitionX < 3.0 / 5.0);

        }

        while (true) {
            List<Recognition> tfodRecognitions = getTfodDetections();

            if (tfodRecognitions.size() == 0) {
                mecanum.driveRobotCentric(0, 0.5, 0);
            } else {
                recognition = tfodRecognitions.stream()
                        .sorted(Comparator.comparingDouble(Recognition::getBottom))
                        .collect(Collectors.toList()).get(0);

                boolean isDone = driveToTfodRecognition(recognition);

                if (isDone) {
                    break;
                }
            }

            arm.lower();
            linearSlide.lower();

            while (!(arm.atSetPoint() && linearSlide.atSetPoint())) {
                arm.moveToSetPoint();
                linearSlide.moveToSetPoint();
                sleep(20);
            }

            grabber.open();
        }
    }

    /**
     * Gets TFOD recognitions.
     *
     * @return the TFOD recognitions that are in WANTED_LABELS
     */
    @NonNull
    protected List<Recognition> getTfodDetections() {
        List<Recognition> currentRecognitions = new ArrayList<>();
        for (TfodProcessor model : tfodModels) {
            currentRecognitions.addAll(model.getRecognitions());
        }

        telemetry.addLine(currentRecognitions.size() + " Objects Detected. ");

        currentRecognitions = currentRecognitions.stream()
                .filter((recognition) -> Arrays.asList(WANTED_LABELS).contains(recognition.getLabel()))
                .collect(Collectors.toList());

        return currentRecognitions;
    }

    protected boolean driveToTfodRecognition(@NonNull Recognition recognition) {
        mecanum.driveRobotCentric(0, recognition.getBottom() / CAMERA_SIZE.getHeight(), 0);

        return CAMERA_SIZE.getHeight() - recognition.getBottom() <= TFOD_RECOGNITION_SCREEN_POS;
    }

    public enum PixelSide {
        LEFT,
        RIGHT,
        CENTER
    }
}
