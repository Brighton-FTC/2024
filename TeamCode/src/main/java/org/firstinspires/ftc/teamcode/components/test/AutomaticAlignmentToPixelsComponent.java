package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Prototype for aligning robot to a stack of pixels (untested). <br />
 * It's meant to go to a specified pixel stack. <br />
 *
 * <b>
 * Note: currently this is using component classes that don't exist in the same branch as this one. <br />
 * Make sure to merge the arm, linear slide, and grabber components into the branch that this is being used in before using this.
 * </b> <br /> <br />
 *
 * <p>
 * Call {@link #startMoving(int, int) startMoving()} once to get the robot to move, and then call {@link #moveRobot()} continuously. <br />
 * <p>
 * You can find out whether the robot is moving using {@link #isMoving()}.
 * </p>
 */

public class AutomaticAlignmentToPixelsComponent {
    // TODO: fine tune these by testing
    public static final double DISTANCE_ERROR = 6; // in inches
    public static final double ANGLE_ERROR = 20; // in degrees

    public static final double MOVING_DESIRED_DISTANCE = 18; // in inches
    public static final double SHIFTING_DESIRED_DISTANCE = 6; // in inches
    public static final double FINISHED_DESIRED_DISTANCE = 12; // in inches

    public static final double MAX_AUTO_SPEED = 0.5;
    public static final double MAX_AUTO_TURN = 0.3;

    public static final double AUTO_DRIVING_DIVISOR = 6;
    public static final double AUTO_ANGLE_DIVISOR = 45;

    public static final int N_PIXEL_STACKS = 6;
    public static final double[] X_OFFSET_FROM_APRIL_TAGS = {6, 18, 30, -30, -18, -6}; // 12 inch spacing between stacks

    // TODO: check these are the right way round
    public static final int LEFT_APRIL_TAG_ID = 7;
    public static final int RIGHT_APRIL_TAG_ID = 10;

    private final ArmComponent arm;
    private final ActiveIntakeComponent activeIntake;
    private final MecanumDrive mecanum;
    private final DistanceSensor distanceSensor;
    private final IMU imu;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private State currentState = State.IDLE;
    private AprilTagDetection aprilTagDetection;

    private int pixelStackIndex = 0;
    private int nPixels;

    /**
     * Code to align the robot to a stack of pixels.
     *
     * @param arm          The {@link ArmComponent} that the robot uses.
     * @param activeIntake The {@link ActiveIntakeComponent} that the robot uses.
     * @param mecanum      The {@link MecanumDrive} for the robot.
     * @param webcam       The webcam for the robot.
     */
    public AutomaticAlignmentToPixelsComponent(ArmComponent arm,
                                               ActiveIntakeComponent activeIntake,
                                               MecanumDrive mecanum,
                                               DistanceSensor distanceSensor,
                                               IMU imu,
                                               WebcamName webcam) {
        this.arm = arm;
        this.activeIntake = activeIntake;
        this.mecanum = mecanum;
        this.distanceSensor = distanceSensor;
        this.imu = imu;

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .enableLiveView(false)
                .build();

        aprilTag = new AprilTagProcessor.Builder().build();
    }

    /**
     * @return If the robot is moving (if it is aligning to a pixel stack).
     */
    public boolean isMoving() {
        return currentState != State.IDLE;
    }

    /**
     * Make the robot stop moving.
     */
    public void stopMoving() {
        currentState = State.IDLE;
        visionPortal.stopStreaming();
    }

    /**
     * If the robot isn't moving, then make it move.
     *
     * @param pixelStackIndex The index of the pixel stack to move to (starting from the left), from the perspective of facing towards the audience side.
     * @param nPixels         The number of pixels to pick up (1 or 2).
     */
    public void startMoving(int pixelStackIndex, int nPixels) {
        if (pixelStackIndex >= N_PIXEL_STACKS || pixelStackIndex < 0) {
            throw new IllegalArgumentException("'pixelStackIndex' must be between 0 and " + N_PIXEL_STACKS + " but is " + pixelStackIndex);
        }

        if (nPixels != 1 && nPixels != 2) {
            throw new IllegalArgumentException("'nPixels' must be either 1 or 2, but was" + nPixels);
        }

        if (currentState == State.IDLE) {
            this.nPixels = nPixels;

            this.pixelStackIndex = pixelStackIndex;
            currentState = State.SEARCHING_FOR_APRIL_TAGS;
        }
    }

    /**
     * If the robot is idle, then make it move, otherwise make it stop moving.
     *
     * @param pixelStackIndex The pixel stack that the robot should go to if it starts moving.
     */
    public void toggleMoving(int pixelStackIndex, int nPixels) {
        if (isMoving()) {
            stopMoving();
        } else {
            startMoving(pixelStackIndex, nPixels);
        }
    }

    /**
     * @return The current state.
     */
    public State getCurrentState() {
        return currentState;
    }

    /**
     * Call continuously once {@link #startMoving(int, int) startMoving()} has been called, to make the robot automatically pick up a pixel.
     */
    public void moveRobot() {
        boolean hasFinished;

        switch (currentState) {
            case SEARCHING_FOR_APRIL_TAGS:
                aprilTagDetection = getAprilTag();
                imu.resetYaw();

                if (aprilTagDetection != null) {
                    if (aprilTagDetection.metadata.id == RIGHT_APRIL_TAG_ID && pixelStackIndex <= 2) {
                        mecanum.driveRobotCentric(0, 0, -MAX_AUTO_TURN);
                    } else if (aprilTagDetection.metadata.id == LEFT_APRIL_TAG_ID && pixelStackIndex >= 3) {
                        mecanum.driveRobotCentric(0, 0, MAX_AUTO_TURN);
                    } else { // if the robot is facing towards the correct april tag
                        currentState = State.MOVING_TO_PIXEL_STACK;
                    }
                }

                break;

            case MOVING_TO_PIXEL_STACK:
                hasFinished = moveToPosition(aprilTagDetection.ftcPose, X_OFFSET_FROM_APRIL_TAGS[pixelStackIndex]);

                if (hasFinished) {
                    aprilTagDetection = null;
                    currentState = State.SHIFTING_TO_PIXEL_STACK;
                }

                break;

            case SHIFTING_TO_PIXEL_STACK:
                hasFinished = driveToDistance(SHIFTING_DESIRED_DISTANCE);

                if (hasFinished) {
                    currentState = State.PREPARING_FOR_PIXEL_PICKUP;
                }

                break;

            case PREPARING_FOR_PIXEL_PICKUP:
                arm.setState(ArmComponent.State.GROUND);

                arm.moveToSetPoint();

                if (arm.atSetPoint()) {
                    currentState = State.PICKING_UP_PIXEL;
                }

                break;

            case PICKING_UP_PIXEL:
                arm.setState(ArmComponent.State.GROUND);

                arm.moveToSetPoint();

                arm.read();

                if (arm.atSetPoint()) {
                    if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                        if (nPixels == 0) {
                            currentState = State.MOVING_BACK_FROM_PIXELS;
                        }

                        activeIntake.turnManually();
                        nPixels--;
                    }

                    activeIntake.moveMotor();
                }

                break;

            case MOVING_BACK_FROM_PIXELS:
                hasFinished = driveToDistance(FINISHED_DESIRED_DISTANCE);

                if (hasFinished) {
                    currentState = State.IDLE;
                }
        }
    }

    /**
     * Gets the april tag that the robot should use for its pathfinding.
     * If there are more than one april tags on the screen, then it will chose the first one returned.
     *
     * @return The first april tag found. If there are no april tags with metadata found, then it will return null.
     */
    @Nullable
    private AprilTagDetection getAprilTag() {
        // get the detections and filter out the ones without metadata
        List<AprilTagDetection> aprilTagDetections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .filter(detection -> detection.metadata.id == LEFT_APRIL_TAG_ID || detection.metadata.id == RIGHT_APRIL_TAG_ID)
                .collect(Collectors.toList());

        if (aprilTagDetections.size() == 0) {
            return null;
        } else {
            return aprilTagDetections.get(0); // TODO: if multiple april tags is a problem, replace this with something more rigorous
        }
    }

    /**
     * Moves the robot to a position relative to the april tag.
     *
     * @param anchor  The april tag that the robot must move to.
     * @param xOffset The horizontal offset from the april tag, in inches.
     *                For example, if you want to move the robot half a foot right of an april tag, then pass in 6.
     * @return If it has finished moving the robot.
     * TODO: improve this
     */
    private boolean moveToPosition(@NonNull AprilTagPoseFtc anchor, double xOffset) {
        // calculate how far the robot has to turn/move
        double headingError = 90 - Math.toDegrees(Math.atan2(anchor.y, anchor.x + xOffset));
        double distanceError = distanceSensor.getDistance(DistanceUnit.INCH);

        boolean hasTurned = turnToAngle(headingError);

        if (hasTurned && !(Math.abs(distanceError - MOVING_DESIRED_DISTANCE) <= DISTANCE_ERROR)) {
            driveToDistance(distanceError);

        } else {
            return true;
        }

        return false;
    }

    /**
     * Turns robot to angle.
     * Note that angle should be -179 to 180 rather than 0 to 360.
     *
     * @return whether it's done turning
     */

    private boolean turnToAngle(double angle) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        currentHeading = currentHeading > 180 ? currentHeading - 360 : currentHeading;

        if (Math.abs(currentHeading - angle) <= ANGLE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0, 0,
                    Range.clip(MAX_AUTO_TURN * (currentHeading - angle) / AUTO_ANGLE_DIVISOR, -MAX_AUTO_TURN, MAX_AUTO_TURN));

            return false;
        }
    }

    /**
     * Drives the robot to the specified distance, in inches.
     *
     * @param distance The distance that the robot must drive to.
     * @return If it has finished moving the robot.
     */
    private boolean driveToDistance(double distance) {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if (Math.abs(currentDistance - distance) <= DISTANCE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0,
                    Range.clip(MAX_AUTO_SPEED * (distance - currentDistance) / AUTO_DRIVING_DIVISOR,
                            -MAX_AUTO_SPEED, MAX_AUTO_SPEED), 0);
            return false;
        }
    }

    /**
     * Possible states for the robot.
     */
    public enum State {
        IDLE("Idle"),
        SEARCHING_FOR_APRIL_TAGS("Searching for april tags"),
        MOVING_TO_PIXEL_STACK("Moving to stack"),
        SHIFTING_TO_PIXEL_STACK("Shifting to  stack"),
        PREPARING_FOR_PIXEL_PICKUP("Preparing for pixel pickup"),
        PICKING_UP_PIXEL("Picking up pixel"),
        MOVING_BACK_FROM_PIXELS("Moving back from pixel stack");

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
