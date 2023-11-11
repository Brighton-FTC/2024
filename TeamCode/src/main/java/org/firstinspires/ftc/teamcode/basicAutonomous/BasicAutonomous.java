package org.firstinspires.ftc.teamcode.basicAutonomous;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Rudimentary autonomous code.
 */
@Disabled
@Autonomous(name = "Basic Autonomous", group = "autonomous-test")
public class BasicAutonomous extends OpMode {
    // TODO: once the custom model exists, replace this with the custom model name
    public String TFOD_MODEL_ASSET = "CenterStage.tflite";

    // TODO: once the custom model exists, modify this
    public String[] LABELS = {
            "Pixel"
    };

    public String[] WANTED_LABELS = {
            "Pixel"
    };

    // TODO: fine tune these values
    public double ARM_ERROR = 15;
    public double LINEAR_SLIDE_ERROR = 15;
    public double ANGLE_ERROR = 20;
    public int VISION_ERROR = 20;

    public double DRIVING_TO_BACKDROP_DIST = 12;
    public double SHIFTING_TO_BACKDROP_DIST = 3;

    public double DRIVE_DIVISOR = 12;
    public double ANGLE_DIVISOR = 90;
    public double STRAFE_DIVISOR = 24;

    public double ARM_DOWN_POS = 0;
    public double ARM_UP_POS = 180;

    public double LINEAR_SLIDE_DOWN_POS = 0;
    public double LINEAR_SLIDE_UP_POS = 180;

    public double GRABBER_CLOSED_POS = 0;
    public double GRABBER_OPEN_POS = 90;

    public double GRABBER_TILTED_DOWN_POS = 0;
    public double GRABBER_TILTED_UP_POS = 90;

    public double MIN_DISTANCE_FROM_OBJECT = 6;

    public double PARKING_DIST_ERROR = 3;

    protected VisionPortal visionPortal;

    protected TfodProcessor tfod;
    protected AprilTagProcessor aprilTag;

    // TODO: set these values
    protected Size cameraSize = new Size(640, 480);

    protected MecanumDrive mecanum;

    protected MotorEx armMotor;
    protected MotorEx linearSlideMotor;

    protected ServoEx grabberServo;
    protected ServoEx grabberTiltServo;

    protected SensorDistanceEx distanceSensor;
    protected GyroEx gyro;

    // TODO: tune these
    protected PIDFController armPidf = new PIDFController(0, 0, 0, 0);
    protected PIDFController linearSlidePidf = new PIDFController(0, 0, 0, 0);

    protected State currentState = State.DRIVING_TO_SPIKE_MARKS;
    protected CorrectSpikeMark correctSpikeMark = null;

    protected TeamColor teamColor = null; // fill this in

    @Override
    public void init() {
        // hardware
        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "front_left"),
                new Motor(hardwareMap, "front_right"),
                new Motor(hardwareMap, "back_left"),
                new Motor(hardwareMap, "back_right")
        );

        armMotor = new MotorEx(hardwareMap, "arm_motor");
        armMotor.setPositionTolerance(ARM_ERROR);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_motor");
        linearSlideMotor.setPositionTolerance(LINEAR_SLIDE_ERROR);
        linearSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        grabberServo = new SimpleServo(hardwareMap, "grabber_servo", GRABBER_CLOSED_POS, GRABBER_OPEN_POS);
        grabberTiltServo = new SimpleServo(hardwareMap, "grabber_tilt_servo", GRABBER_TILTED_DOWN_POS, GRABBER_TILTED_UP_POS);

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor");
        gyro = new RevIMU(hardwareMap, "gyro");

        // vision stuff
        aprilTag = new AprilTagProcessor.Builder().build();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                // I have no idea what the following two statements do; find that out asap
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
        switch (currentState) {
            case DRIVING_TO_SPIKE_MARKS:
                List<Recognition> recognitions = getTfodDetections();

                if (recognitions.size() > 0) {
                    Recognition currentRecognition = recognitions.stream()
                            .min(Comparator.comparingDouble(Recognition::getTop))
                            .get();

                    boolean isDone = driveToTfodObject(currentRecognition, cameraSize.getWidth() * 2 / 3, cameraSize.getHeight() / 2);

                    if (isDone) {
                        if ((currentRecognition.getLeft() + currentRecognition.getRight()) / 2 < cameraSize.getWidth() / 3.0) {
                            correctSpikeMark = CorrectSpikeMark.LEFT;
                        } else if ((currentRecognition.getLeft() + currentRecognition.getRight()) / 2 > cameraSize.getWidth() * 2.0 / 3.0) {
                            correctSpikeMark = CorrectSpikeMark.RIGHT;
                        } else {
                            correctSpikeMark = CorrectSpikeMark.CENTER;
                        }

                        currentState = State.PLACING_PURPLE_PIXEL;
                    }
                } else {
                    if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
                        mecanum.driveRobotCentric(0, 1, 0);
                    }
                }

                break;

            case PLACING_PURPLE_PIXEL:
                armPidf.setSetPoint(ARM_DOWN_POS);
                linearSlidePidf.setSetPoint(LINEAR_SLIDE_DOWN_POS);

                armMotor.set(armPidf.calculate());
                linearSlideMotor.set(linearSlidePidf.calculate());

                grabberTiltServo.turnToAngle(GRABBER_TILTED_DOWN_POS);

                if (armPidf.atSetPoint() && linearSlidePidf.atSetPoint()) {
                    grabberServo.turnToAngle(GRABBER_OPEN_POS);
                    currentState = State.DONE;
                    gyro.reset();
                }

            case TURNING_TO_BACKDROP:
                boolean isDone = turnToAngle(-90);

                if (isDone) {
                    currentState = State.DRIVING_TO_BACKDROP;
                }

            case DRIVING_TO_BACKDROP:
                if (distanceSensor.getDistance(DistanceUnit.INCH) > DRIVING_TO_BACKDROP_DIST) {
                    mecanum.driveRobotCentric(0, 1, 0);
                } else {
                    currentState = State.SHIFTING_TO_BACKDROP;
                }

            case PREPARING_FOR_SHIFTING:
                armPidf.setSetPoint(ARM_UP_POS);
                armMotor.set(armPidf.calculate());

                linearSlidePidf.setSetPoint(LINEAR_SLIDE_UP_POS);
                linearSlideMotor.set(linearSlidePidf.calculate());

                grabberTiltServo.turnToAngle(GRABBER_TILTED_UP_POS);
                break;

            case SHIFTING_TO_BACKDROP:
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
                grabberServo.turnToAngle(GRABBER_OPEN_POS);
                currentState = State.STRAFING_TO_PARK;
                gyro.reset();
                break;

            case STRAFING_TO_PARK:
                mecanum.driveRobotCentric(0, teamColor == TeamColor.RED ? 0.5 : -0.5, 0);

                if (distanceSensor.getDistance(DistanceUnit.INCH) > SHIFTING_TO_BACKDROP_DIST + PARKING_DIST_ERROR) {
                    currentState = State.PARKING;
                }
                break;

            case PARKING:
                if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
                    mecanum.driveRobotCentric(0, 0.5, 0);
                } else {
                    currentState = State.DONE;
                }
        }
    }

    @NonNull
    protected List<Recognition> getTfodDetections() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addLine(currentRecognitions.size() + " Objects Detected. ");

        currentRecognitions = currentRecognitions.stream()
                .filter((recognition) -> Arrays.asList(WANTED_LABELS).contains(recognition))
                .collect(Collectors.toList());

        return currentRecognitions;
    }

    @NonNull
    protected List<AprilTagDetection> getAprilTagDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addLine(currentDetections.size() + " AprilTags Detected. ");

        currentDetections = currentDetections.stream()
                .filter((detection) -> detection.metadata != null)
                .collect(Collectors.toList());

        return currentDetections;
    }

    protected boolean turnToAngle(double angle) {
        double heading = gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();

        if (Math.abs(angle - heading) <= ANGLE_ERROR) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, 0, heading / ANGLE_DIVISOR);
            return false;
        }
    }

    protected boolean driveToTfodObject(@NonNull Recognition object, int xPos, int yPos) {
        if (Math.abs(object.estimateAngleToObject(AngleUnit.DEGREES)) <= ANGLE_ERROR) {
            gyro.reset();
            turnToAngle(object.estimateAngleToObject(AngleUnit.DEGREES));
            return false;

        } else if (Math.abs((object.getTop() + object.getBottom()) / 2 - yPos) <= VISION_ERROR) {
            if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
                mecanum.driveRobotCentric(0, (object.getTop() + object.getBottom() / (2 * cameraSize.getHeight())), 0);
            }
            return false;

        } else if (Math.abs(object.getLeft() + object.getRight() / 2 - xPos) <= VISION_ERROR) {
            mecanum.driveRobotCentric(0, 0, (object.getTop() + object.getBottom() / (2 * cameraSize.getWidth())));

            return false;

        } else {
            return true;
        }
    }

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