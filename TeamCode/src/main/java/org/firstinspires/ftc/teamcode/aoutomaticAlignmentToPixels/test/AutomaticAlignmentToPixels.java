package org.firstinspires.ftc.teamcode.aoutomaticAlignmentToPixels.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Prototype for aligning robot to a stack of pixels (<b>untested and in development</b>). <br />
 * It's meant to go to a specified pixel stack when goToPixelStack() is called continuously, and startMoving() has also been called. <br />
 * <i>Warning: make sure that the robot is angled towards the left april tag for the first 3 pixel stacks, and the right april tag for the last 3.
 * Otherwise, the robot will inevitably crash into a wall. </i> <br />
 *
 * An example of how to implement this is shown in the {@link AutomaticAlignmentToPixelsTestOpMode} class.
 */

public class AutomaticAlignmentToPixels {
    // TODO: fine tune these by testing
    public static final double DISTANCE_ERROR = 6; // in inches
    public static final double ANGLE_ERROR = 20; // in degrees
    public static final double LINEAR_SLIDE_ERROR = 15;

    public static final double MOVING_DESIRED_DISTANCE = 18; // in inches
    public static final double SHIFTING_DESIRED_DISTANCE = 6; // in inches

    public static final double MAX_AUTO_SPEED = 0.5;
    public static final double MAX_AUTO_TURN = 0.3;
    public static final double MAX_LINEAR_SLIDE_SPEED = 0.5;

    public static final int LINEAR_SLIDE_DOWN_POS = 0;
    public static final int LINEAR_SLIDE_UP_POS = 2000;

    public static final double GRABBER_CLOSED_POS = 0;
    public static final double GRABBER_OPEN_POS = 90;

    public static final double GRABBER_TILTED_DOWN_POS = 0;
    public static final double GRABBER_TILTED_UP_POS = 90;

    public static final double SCANNING_FOR_PIXEL_STACK_ANGLE = 30; // degrees

    public static final double AUTO_DRIVING_DIVISOR = 6;
    public static final double AUTO_ANGLE_DIVISOR = 45;

    public static final double kP = 0.05;

    public static AprilTagProcessor aprilTag;
    public static VisionPortal visionPortal;

    public static MecanumDrive mecanum;

    public static MotorEx linearSlideMotor;
    public static ServoEx grabberServo;
    public static ServoEx grabberTiltServo;

    public static SensorDistanceEx distanceSensor;
    public static GyroEx gyro;
    public static TouchSensor touchSensor;

    public static final int N_PIXEL_STACKS = 6;
    public static final double[] xOffsetFromAprilTags = {0, 11, 22, -22, -11, 0}; // 11 inch spacing between stacks

    public static double[] closestAngle = {0, Double.POSITIVE_INFINITY}; // [angle (degrees), distance (inches)]

    public static State currentState = State.IDLE;

    public static AprilTagDetection aprilTagDetection;

    /**
     * Call in the init() method of an opmode, or right at the start of the runOpMode() method of a linear opmode.
     * @param hardwareMap The hardware map.
     */
    public static void init(@NonNull HardwareMap hardwareMap) {
        // TODO: fill in component names

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .addProcessor(aprilTag)
                .build();

        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "front_left"),
                new Motor(hardwareMap, "front_right"),
                new Motor(hardwareMap, "back_left"),
                new Motor(hardwareMap, "back_right")
        );

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_name", LINEAR_SLIDE_DOWN_POS, LINEAR_SLIDE_UP_POS);
        linearSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        linearSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionTolerance(LINEAR_SLIDE_ERROR);
        linearSlideMotor.setPositionCoefficient(kP);

        grabberServo = new SimpleServo(hardwareMap, "grabber_servo_name", GRABBER_CLOSED_POS, GRABBER_OPEN_POS);
        grabberTiltServo = new SimpleServo(hardwareMap, "grabber_tilt_servo_name", GRABBER_TILTED_DOWN_POS, GRABBER_TILTED_UP_POS);

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_name");
        gyro = new RevIMU(hardwareMap, "gyro_name");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor_name");
    }

    /**
     * Call in the stop() method of an opmode, or after the while loop of a linear opmode.
     */
    public static void stop() {
        startMoving();
        visionPortal.close();
    }

    public static boolean isMoving() {
        return currentState != State.IDLE;
    }

    public static void stopMoving() {
        currentState = State.IDLE;
    }

    public static void startMoving() {
        if (currentState == State.IDLE) {
            currentState = State.SEARCHING_FOR_APRIL_TAGS;
        }
    }

    public static State getCurrentState() {
        return currentState;
    }

    /**
     *
     * Call continuously to drive the robot to the specified pixel stack. <br />
     * You must call startMoving() for the robot to actually do stuff. <br />
     * The program will then (hopefully) use april tags to get to a certain distance from the wall,
     * use the distance sensor to get even closer, and then pick up a pixel (still in development). <br />
     * If the robot is closer to the wall, it will just go straight to the nearest pixel stack
     * (needs to be confirmed by JRCs). <br />
     *
     * Note: You must point the robot to the left april tag for the first 3 pixel stacks, and to the right april tag for the last 3.
     * Otherwise the robot will inevitably crash into a wall.
     * @param pixelStackIndex The index of the pixel stack to go to. 0 to 5 (inclusive).
     *
     */
    public static void goToPixelStack(int pixelStackIndex) {
        if (pixelStackIndex < 0 || pixelStackIndex >= N_PIXEL_STACKS) {
            throw new IllegalArgumentException("pixelStackIndex must be between 0 and " + N_PIXEL_STACKS + ".");
        }

        switch (currentState) {
            case IDLE:
                linearSlideMotor.setTargetPosition(LINEAR_SLIDE_DOWN_POS);

                if (!linearSlideMotor.atTargetPosition()) {
                    linearSlideMotor.set(MAX_LINEAR_SLIDE_SPEED);
                } else {
                    linearSlideMotor.set(0);
                }

                break;

            case SEARCHING_FOR_APRIL_TAGS:
                aprilTagDetection = getAprilTag();
                gyro.reset();

                if (aprilTagDetection != null) {
                    currentState = State.MOVING_TO_PIXEL_STACK;
                } else if (distanceSensor.getDistance(DistanceUnit.INCH) <= MOVING_DESIRED_DISTANCE) {
                    currentState = State.TURNING;
                }

                break;

            case MOVING_TO_PIXEL_STACK:
                boolean hasFinished = moveToPosition(aprilTagDetection.ftcPose, xOffsetFromAprilTags[pixelStackIndex]);

                if (hasFinished) {
                    aprilTagDetection = null;
                    currentState = State.TURNING;
                }

                break;

            case TURNING:
                hasFinished = turnToAngle(-SCANNING_FOR_PIXEL_STACK_ANGLE);

                if (hasFinished) {
                    gyro.reset();

                    currentState = State.SEARCHING_FOR_PIXEL_STACK;
                }

                break;

            case SEARCHING_FOR_PIXEL_STACK:
                hasFinished = scanForPixelStack(SCANNING_FOR_PIXEL_STACK_ANGLE * 2, true);

                if (hasFinished) {
                    currentState = State.SHIFTING_TO_PIXEL_STACK;
                }

                break;

            case SHIFTING_TO_PIXEL_STACK:
                hasFinished = driveToDistance(SHIFTING_DESIRED_DISTANCE, DistanceUnit.INCH);

                if (distanceSensor.getDistance(DistanceUnit.INCH) > closestAngle[1]) { // if the robot has driven off at the wrong angle
                    currentState = State.TURNING;
                }

                if (hasFinished) {
                    currentState = State.PREPARING_FOR_PIXEL_PICKUP;
                }

                break;

            case PREPARING_FOR_PIXEL_PICKUP:
                if (!linearSlideMotor.atTargetPosition()) {
                    linearSlideMotor.set(MAX_LINEAR_SLIDE_SPEED);
                }
                grabberServo.turnToAngle(GRABBER_OPEN_POS);
                grabberTiltServo.turnToAngle(GRABBER_TILTED_DOWN_POS);

                if (linearSlideMotor.atTargetPosition()) {
                    currentState = State.PICKING_UP_PIXEL;
                }

                break;

            case PICKING_UP_PIXEL:
                linearSlideMotor.setTargetPosition(LINEAR_SLIDE_DOWN_POS);
                if (!touchSensor.isPressed()) {
                    linearSlideMotor.set(MAX_LINEAR_SLIDE_SPEED / 2);
                } else {
                    grabberServo.turnToAngle(GRABBER_CLOSED_POS);
                    currentState = State.IDLE;
                }
            }
        }

    // gets the april tag that the robot should use for its pathfinding
    // if there are more than one april tags on the screen, then it will chose the first one returned
    @Nullable
    private static AprilTagDetection getAprilTag() {
        // get the detections and filter out the ones without metadata
        List<AprilTagDetection> aprilTagDetections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .collect(Collectors.toList());

        if (aprilTagDetections.size() == 0) {
            return null;
        } else {
            return aprilTagDetections.get(0); // TODO: if multiple april tags is a problem, replace this with something more rigorous
        }
    }

    // attempts to move robot to april tag, and returns if the robot has stopped
    private static boolean moveToPosition(@NonNull AprilTagPoseFtc anchor, double xOffset) {
        // calculate how far the robot has to turn/move
        // idk if I have to use anchor.y or anchor.range here, but am using anchor.y for now
        double headingError = Math.atan2(anchor.y, anchor.x + xOffset) * 180 / Math.PI;
        double distanceError = distanceSensor.getDistance(DistanceUnit.INCH);

        double currentHeading = gyro.getHeading();
        // set the angle to be from -179 to 180 instead of 0 to 359
        currentHeading = currentHeading > 180 ? currentHeading - 360 : currentHeading;

        if (!(Math.abs(headingError - currentHeading) <= ANGLE_ERROR)) {
            turnToAngle(headingError);

        } else if (!(Math.abs(distanceError - MOVING_DESIRED_DISTANCE) <= DISTANCE_ERROR)) {
            driveToDistance(distanceError, DistanceUnit.INCH);

        } else {
            return true;
        }

        return false;
    }

    private static boolean scanForPixelStack(double searchAngle, boolean isClockwise) {
        if (distanceSensor.getDistance(DistanceUnit.INCH) < closestAngle[1]) {
            closestAngle[0] = gyro.getHeading();
            closestAngle[1] = distanceSensor.getDistance(DistanceUnit.INCH);
        }

        return turnToAngle(isClockwise ? searchAngle : -searchAngle);
    }

    // turns robot to angle
    // note that angle should be -179 to 180 rather than 0 to 360
    // returns whether it's done turning

    private static boolean turnToAngle(double angle) {
        double currentHeading = gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();

        if (Math.abs(currentHeading - angle) <= ANGLE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0, 0,
                    Range.clip(MAX_AUTO_TURN * (currentHeading - angle) / AUTO_ANGLE_DIVISOR, -MAX_AUTO_TURN, MAX_AUTO_TURN));

            return false;
        }
    }

    // drives robot forwards to distance
    // returns whether it's done driving
    private static boolean driveToDistance(double distance, DistanceUnit distanceUnit) {
        double currentDistance = distanceSensor.getDistance(distanceUnit);

        if (Math.abs(currentDistance - distance) <= DISTANCE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0,
                    Range.clip(MAX_AUTO_SPEED * (distance - currentDistance) / distanceUnit.fromInches(AUTO_DRIVING_DIVISOR),
                            -MAX_AUTO_SPEED, MAX_AUTO_SPEED), 0);
            return false;
        }
    }

    public enum State {
        IDLE("Idle"),
        SEARCHING_FOR_APRIL_TAGS("Searching for april tags"),
        MOVING_TO_PIXEL_STACK("Moving to stack"),
        TURNING("Turning"),
        SEARCHING_FOR_PIXEL_STACK("Searching for stack"),
        SHIFTING_TO_PIXEL_STACK("Shifting to  stack"),
        PREPARING_FOR_PIXEL_PICKUP("Preparing for pixel pickup"),
        PICKING_UP_PIXEL("Picking up pixel");

        private final String stringRepr;

        State(String stringRepr) {
            this.stringRepr = stringRepr;
        }


        @NonNull
        @Override
        public String toString() {
            return this.stringRepr;
        }
    }
}