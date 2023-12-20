package org.firstinspires.ftc.teamcode.opMode.test.withoutDistanceSensor;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

/**
 * Rudimentary autonomous code. <br />
 * Moves to the Team Prop, places down the purple pixel, drives to the backdrop, and places the yellow pixel. <br />
 * <b>
 * Note: this code uses method calls from component classes in other branches.
 * Be sure to merge the grabber-code, linear-slide-code, and arm-code branches into the branch that this is in before running.
 * </b>
 */
public class BasicAutonomousGeneric extends OpMode {
    // TODO: once the custom model exists, replace this with the custom model name
    public static final String WHITE_PIXELS_MODEL_ASSET = "CenterStage.tflite";
    public static final String TEAM_PROP_MODEL_ASSET = "TeamPropModel_v1.0.tflite";

    // TODO: once the custom model exists, modify this
    public static final String[] WHITE_PIXEL_LABELS = {"Pixel"};
    public static final String[] TEAM_PROP_LABELS = {"red_pixel", "blue_pixel"};

    public String[] WANTED_LABELS = {
            "Pixel"
    };

    // TODO: fine tune these values
    public static final double ANGLE_ERROR = 20;
    public static final int VISION_ERROR = 20;

    public static final double DRIVING_TO_BACKDROP_DIST = 12;
    public static final double SHIFTING_TO_BACKDROP_DIST = 3;

    public static final double DRIVE_DIVISOR = 12;
    public static final double ANGLE_DIVISOR = 90;
    public static final double STRAFE_DIVISOR = 24;

    public static final double PARKING_DRIVE_DISTANCE = 12;
    public static final double[] PARKING_STRAFE_DISTANCES = {18, 24, 30}; // in inches

    public static final double MECANUM_DPP = 18; // distance per pulse, in mm per tick

    public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5)
    );

    public static final double CAMERA_Y_POS = 2.0 / 3.0;

    protected VisionPortal visionPortal;

    protected TfodProcessor[] tfodModels;
    protected AprilTagProcessor aprilTag;

    // TODO: set these values
    protected Size cameraSize = new Size(640, 480);

    protected Motor[] mecanumMotors;
    protected MecanumDrive mecanum;

    protected ArmComponent arm;
    protected LinearSlideComponent linearSlide;
    protected GrabberComponent grabber;

    protected GyroEx gyro;

    protected State currentState = State.DRIVING_TO_SPIKE_MARKS;
    protected CorrectSpikeMark correctSpikeMark = null;

    protected MecanumDriveOdometry odometry;

    protected ElapsedTime elapsedTime = new ElapsedTime();

    protected TeamColor teamColor = null; // fill this in in the color specific opmode
    protected double backdropTurningAngle = 0; // fill this in as well

    @Override
    public void init() {
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
            motor.setDistancePerPulse(MECANUM_DPP);
            motor.setInverted(!motor.getInverted());
        }

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linear_slide_motor"));
        grabber = new GrabberComponent(new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360),
                new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360));

        gyro = new RevIMU(hardwareMap, "gyro");

        odometry = new MecanumDriveOdometry(MECANUM_KINEMATICS, Rotation2d.fromDegrees(0), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

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
                .setCameraResolution(cameraSize)
                .enableLiveView(true)
                .addProcessors(tfodModels)
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void loop() {
        arm.read();
        linearSlide.read();

        switch (currentState) {
            case DRIVING_TO_SPIKE_MARKS:
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

                        currentState = State.PLACING_PURPLE_PIXEL;
                    }
                }

                break;

            case STRAFING_FOR_PURPLE_PIXEL_PLACEMENT:
                recognitions = getTfodDetections();

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

                        currentState = State.PLACING_PURPLE_PIXEL;
                    }
                }

                break;

            case PLACING_PURPLE_PIXEL:
                arm.moveToSetPoint();
                linearSlide.moveToSetPoint();

                if (arm.atSetPoint() && linearSlide.atSetPoint()) {
                    grabber.open();
                    currentState = State.TURNING_TO_BACKDROP;
                    gyro.reset();
                }

            case TURNING_TO_BACKDROP:
                boolean isDone = turnToAngle(backdropTurningAngle);

                if (isDone) {
                    currentState = State.DRIVING_TO_BACKDROP;
                }

            case DRIVING_TO_BACKDROP:
                AprilTagDetection currentDetection = null;
                try {
                    if (teamColor == TeamColor.RED) {
                        currentDetection = getAprilTagDetections().stream()
                                .filter((detection) -> detection.id == correctSpikeMark.getRedAprilTagId())
                                .collect(Collectors.toList())
                                .get(0);

                    } else if (teamColor == TeamColor.BLUE) {
                        currentDetection = getAprilTagDetections().stream()
                                .filter((detection) -> detection.id == correctSpikeMark.getBlueAprilTagId())
                                .collect(Collectors.toList())
                                .get(0);
                    }
                } catch (IndexOutOfBoundsException e) {
                    mecanum.driveRobotCentric(0, 1, 0);
                    break;
                }

                assert currentDetection != null;
                if (currentDetection.ftcPose.y < DRIVING_TO_BACKDROP_DIST) {
                    arm.lift();
                    linearSlide.lift();
                    currentState = State.PREPARING_FOR_SHIFTING;
                }
                else {
                    mecanum.driveRobotCentric(0, 1, 0);
                    break;
                }

                break;

            case PREPARING_FOR_SHIFTING:
                arm.moveToSetPoint();
                linearSlide.moveToSetPoint();

                if (arm.atSetPoint() && linearSlide.atSetPoint()) {
                    currentState = State.SHIFTING_TO_BACKDROP;
                }

                break;

            case SHIFTING_TO_BACKDROP:
                currentDetection = null;
                try {
                    if (teamColor == TeamColor.RED) {
                        currentDetection = getAprilTagDetections().stream()
                                .filter((detection) -> detection.id == correctSpikeMark.getRedAprilTagId())
                                .collect(Collectors.toList())
                                .get(0);

                    } else if (teamColor == TeamColor.BLUE) {
                        currentDetection = getAprilTagDetections().stream()
                                .filter((detection) -> detection.id == correctSpikeMark.getBlueAprilTagId())
                                .collect(Collectors.toList())
                                .get(0);
                    }

                    assert currentDetection != null;
                    boolean hasShifted = shiftToAprilTag(currentDetection);

                    if (hasShifted) {
                        currentState = State.PLACING_YELLOW_PIXEL;
                    }

                } catch (IndexOutOfBoundsException e) {
                    currentState = State.STRAFING_TO_PARK;

                    break;
                }

            case PLACING_YELLOW_PIXEL:
                grabber.open();

                currentState = State.STRAFING_TO_PARK;

                odometry.resetPosition(new Pose2d(), new Rotation2d());
                elapsedTime.reset();

                gyro.reset();
                break;

            case STRAFING_TO_PARK:
                mecanum.driveRobotCentric(0, teamColor == TeamColor.RED ? 0.5 : -0.5, 0);
                odometry.updateWithTime(elapsedTime.time(TimeUnit.SECONDS),
                        Rotation2d.fromDegrees(gyro.getHeading()),
                        new MecanumDriveWheelSpeeds(
                                mecanumMotors[0].getRate(),
                                mecanumMotors[1].getRate(),
                                mecanumMotors[2].getRate(),
                                mecanumMotors[3].getRate()
                        )
                );

                int aprilTagIndex;

                if ((correctSpikeMark == CorrectSpikeMark.LEFT && teamColor == TeamColor.RED)
                        || (correctSpikeMark == CorrectSpikeMark.RIGHT && teamColor == TeamColor.BLUE)) {
                    aprilTagIndex = 0;
                } else if ((correctSpikeMark == CorrectSpikeMark.RIGHT && teamColor == TeamColor.RED)
                || (correctSpikeMark == CorrectSpikeMark.LEFT && teamColor == TeamColor.BLUE)) {
                    aprilTagIndex = 2;
                } else {
                    aprilTagIndex = 1;
                }

                if (Math.abs(odometry.getPoseMeters().getY()) >= PARKING_STRAFE_DISTANCES[aprilTagIndex]) {
                    currentState = State.PARKING;
                    break;
                }

            case PARKING:
                mecanum.driveRobotCentric(0, 0.5, 0);

                odometry.updateWithTime(elapsedTime.time(TimeUnit.SECONDS),
                        Rotation2d.fromDegrees(gyro.getHeading()),
                        new MecanumDriveWheelSpeeds(
                                mecanumMotors[0].getRate(),
                                mecanumMotors[1].getRate(),
                                mecanumMotors[2].getRate(),
                                mecanumMotors[3].getRate()
                        )
                );

                if (Math.abs(odometry.getPoseMeters().getX()) >= PARKING_DRIVE_DISTANCE) {
                    currentState = State.DONE;
                }
        }

        telemetry.addData("Current state: ", currentState);
        telemetry.update();
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

    /**
     * Gets April Tags.
     *
     * @return all April Tags with metadata
     */
    @NonNull
    protected List<AprilTagDetection> getAprilTagDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addLine(currentDetections.size() + " AprilTags Detected. ");

        currentDetections = currentDetections.stream()
                .filter((detection) -> detection.metadata != null)
                .collect(Collectors.toList());

        return currentDetections;
    }

    /**
     * Turns the robot.
     *
     * @param angle angle from 0 to 360
     * @return whether the robot is finished turning
     */
    protected boolean turnToAngle(double angle) {
        double heading = gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();

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
            mecanum.driveRobotCentric(0, (object.getTop() + object.getBottom() / (2 * cameraSize.getHeight())), 0);

            return false;

        } else {
            return true;
        }
    }

    /**
     * Shifts to the April Tag (strafes horizontally to line up with the April Tag)
     *
     * @param detection the April Tag that the robot is shifting to
     * @return if the robot has finished shifting.
     */
    protected boolean shiftToAprilTag(@NonNull AprilTagDetection detection) {
        if (Math.abs(detection.ftcPose.range) <= SHIFTING_TO_BACKDROP_DIST) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, detection.ftcPose.y / DRIVE_DIVISOR, detection.ftcPose.x / STRAFE_DIVISOR);
            return false;
        }
    }

    public enum State {
        DRIVING_TO_SPIKE_MARKS,
        STRAFING_FOR_PURPLE_PIXEL_PLACEMENT,
        PLACING_PURPLE_PIXEL,
        TURNING_TO_BACKDROP,
        DRIVING_TO_BACKDROP,
        PREPARING_FOR_SHIFTING,
        SHIFTING_TO_BACKDROP,
        PLACING_YELLOW_PIXEL,
        STRAFING_TO_PARK,
        PARKING,
        DONE
    }

    public enum CorrectSpikeMark {
        LEFT(1, 4),
        CENTER(2, 5),
        RIGHT(3, 6);

        private final int blueAprilTagId;
        private final int redAprilTagId;

        CorrectSpikeMark(int blueAprilTagId, int redAprilTagId) {
            this.blueAprilTagId = blueAprilTagId;
            this.redAprilTagId = redAprilTagId;
        }

        public int getBlueAprilTagId() {
            return blueAprilTagId;
        }

        public int getRedAprilTagId() {
            return redAprilTagId;
        }
    }

    public enum TeamColor {
        BLUE,
        RED
    }
}