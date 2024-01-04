package org.firstinspires.ftc.teamcode.opMode.test;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.AutomaticPixelPlacement;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Rudimentary autonomous code. <br />
 * Moves to the Team Prop, places down the purple pixel, drives to the backdrop, and places the yellow pixel. <br />
 *
 * <b>
 * Note: this code uses method calls from component classes in other branches.
 * Be sure to merge the grabber-code, linear-slide-code, and arm-code branches into the branch that this is in before running.
 * </b>
 */
public class BasicAutonomousGeneric extends OpMode {
    // TODO: once the custom model exists, replace this with the custom model name
    public final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    // TODO: once the custom model exists, modify this
    public final String[] LABELS = {
            "Pixel",
            "red_cone",
            "blue_cone"
    };

    public String[] WANTED_LABELS = {
            "Pixel"
    };

    // TODO: fine tune these values
    public final double ANGLE_ERROR = 20;
    public final int VISION_ERROR = 20;

    public final double SHIFTING_TO_BACKDROP_DIST = 3;

    public final double ANGLE_DIVISOR = 90;

    public final double MIN_DISTANCE_FROM_OBJECT = 6;

    public final double PARKING_DIST_ERROR = 3;

    public final double CAMERA_Y_POS = 2.0 / 3.0;

    protected VisionPortal visionPortal;

    protected TfodProcessor tfod;
    protected AprilTagProcessor aprilTag;

    // TODO: set these values
    protected Size cameraSize = new Size(640, 480);

    protected MecanumDrive mecanum;

    protected ArmComponent arm;
    protected LinearSlideComponent linearSlide;
    protected GrabberComponent grabber;

    protected AutomaticPixelPlacement automaticPixelPlacement;

    protected SensorDistanceEx distanceSensor;
    protected IMU imu;

    protected Runnable currentState = this::driveToSpikeMarks;
    protected CorrectSpikeMark correctSpikeMark = null;

    protected TeamColor teamColor = null; // fill this in in the color specific opmode
    protected double backdropTurningAngle = 0; // fill this in as well

    @Override
    public void init() {
        // hardware

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        mecanum = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        // invert motors (because of hardware)
        for (Motor motor : motors) {
            motor.setInverted(!motor.getInverted());
        }

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linear_slide_motor"), arm);
        grabber = new GrabberComponent(new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360),
                new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360));

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        automaticPixelPlacement = new AutomaticPixelPlacement(arm, linearSlide, grabber, mecanum, aprilTag, imu);

        // vision stuff
        aprilTag = new AprilTagProcessor.Builder().build();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                // TODO: I have no idea what the following two statements do; find that out asap
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        tfod.setMinResultConfidence(0.75F);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .setCameraResolution(cameraSize)
                .enableLiveView(true)
                .addProcessors(tfod, aprilTag)
                .build();
    }

    @Override
    public void loop() {
        arm.read();
        linearSlide.read();

        currentState.run();

        telemetry.update();
    }

    protected void driveToSpikeMarks() {
        telemetry.addLine("Driving to spike marks.");

        List<Recognition> recognitions = getTfodDetections();

        if (recognitions.size() > 0) {
            Recognition currentRecognition = recognitions.stream()
                    .min(Comparator.comparingDouble(Recognition::getTop))
                    .get();

            boolean isDone = driveToTfodObject(currentRecognition, (int) (cameraSize.getHeight() * CAMERA_Y_POS));

            if (isDone) {
                if ((currentRecognition.getLeft() + currentRecognition.getRight()) / 2 < cameraSize.getWidth() / 3.0) {
                    correctSpikeMark = CorrectSpikeMark.LEFT;
                } else if ((currentRecognition.getLeft() + currentRecognition.getRight()) / 2 > cameraSize.getWidth() * CAMERA_Y_POS) {
                    correctSpikeMark = CorrectSpikeMark.RIGHT;
                } else {
                    correctSpikeMark = CorrectSpikeMark.CENTER;
                }

                currentState = this::strafeForPurplePixelPlacement;
            }
        } else {
            if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
                mecanum.driveRobotCentric(0, 1, 0);
            }
        }

    }

    protected void strafeForPurplePixelPlacement() {
        telemetry.addLine("Strafing to place purple pixel.");

        List<Recognition> recognitions = getTfodDetections();

        if (recognitions.size() > 0) {
            Recognition currentRecognition = recognitions.stream()
                    .min(Comparator.comparingDouble(Recognition::getTop))
                    .get();

            // get the pixel centered to between 2/5 and 3/5 of the camera width
            if (currentRecognition.getRight() > cameraSize.getWidth() * 3.0 / 5.0) {
                mecanum.driveRobotCentric(0.5, 0, 0);

            } else if (currentRecognition.getLeft() < cameraSize.getWidth() * 2.0 / 5.0) {
                mecanum.driveRobotCentric(-0.5, 0, 0);

            } else {
                arm.lower();
                linearSlide.lower();

                currentState = this::placePurplePixel;
            }
        }
    }

    protected void placePurplePixel() {
        telemetry.addLine("Placing purple pixel");

        automaticPixelPlacement.placeOnGround();

        if (!automaticPixelPlacement.isPlacingPixel()) {
            currentState = this::turnToBackdrop;
            imu.resetYaw();
        }
    }

    protected void turnToBackdrop() {
        telemetry.addLine("Turning to backdrop. ");

        boolean isDone = turnToAngle(backdropTurningAngle);

        if (isDone) {
            currentState = this::placeYellowPixel;
            automaticPixelPlacement.placeOnBackdrop();
        }
    }

    protected void placeYellowPixel() {
        telemetry.addLine("Placing yellow pixel. ");

        automaticPixelPlacement.run();

        if (!automaticPixelPlacement.isPlacingPixel()) {
            currentState = this::strafeToPark;
        }
    }

    protected void strafeToPark() {
        telemetry.addLine("Strafing to park. ");

        mecanum.driveRobotCentric(0, teamColor == TeamColor.RED ? 0.5 : -0.5, 0);

        if (distanceSensor.getDistance(DistanceUnit.INCH) > SHIFTING_TO_BACKDROP_DIST + PARKING_DIST_ERROR) {
            currentState = this::park;
        }
    }

    protected void park() {
        telemetry.addLine("Parking. ");

        if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
            mecanum.driveRobotCentric(0, 0.5, 0);
        } else {
            currentState = () -> {};
        }
    }

    /**
     * Gets TFOD recognitions.
     *
     * @return the TFOD recognitions that are in WANTED_LABELS
     */
    @NonNull
    protected List<Recognition> getTfodDetections() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addLine(currentRecognitions.size() + " Objects Detected. ");

        currentRecognitions = currentRecognitions.stream()
                .filter((recognition) -> Arrays.asList(WANTED_LABELS).contains(recognition.getLabel()))
                .collect(Collectors.toList());

        return currentRecognitions;
    }

    /**
     * Turns the robot.
     *
     * @param angle angle from -180 to 180.
     * @return whether the robot is finished turning.
     */
    protected boolean turnToAngle(double angle) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (Math.abs(angle - heading) <= ANGLE_ERROR) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, 0, heading / ANGLE_DIVISOR);
            return false;
        }
    }

    /**
     * Drives to the TFOD object.
     * Uses the y position of the object and the distance sensor to determine if it is close enough.
     *
     * @param object the TFOD recognition
     * @param yPos   The y position that the TFOD object should be at.
     * @return if the robot has finished driving yet
     */
    protected boolean driveToTfodObject(@NonNull Recognition object, int yPos) {
        if (Math.abs((object.getTop() + object.getBottom()) / 2 - yPos) <= VISION_ERROR) {
            if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
                mecanum.driveRobotCentric(0, (object.getTop() + object.getBottom() / (2 * cameraSize.getHeight())), 0);
            }

            return false;

        } else {
            return true;
        }
    }

    public enum CorrectSpikeMark {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum TeamColor {
        BLUE,
        RED
    }
}