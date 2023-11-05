package org.firstinspires.ftc.teamcode.aoutomaticAlignmentToPixels.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * It's meant to go to a specified pixel stack when an input is given <br />
 * <i>Warning: make sure that the robot is angled towards the left april tag for the first 3 pixel stacks, and the right april tag for the last 3.
 * Otherwise, the robot will inevitably crash into a wall. </i>
 *
 * <hr />
 *
 * Controls:
 * <ul>
 *     <li>X - Start going to a cone stack (press again to cancel). </li>
 *     <li>DPAD LEFT and DPAD RIGHT - change the pixel stack that the robot is meant to be going to. </li>
 * </ul>
 *
 * <hr />
 *
 * Stuff to do:
 * <ul>
 *     <li>Implement the code for picking up a pixel. </li>
 *     <li>Find a better way of knowing when the robot has arrived in front of the desired pixel stack. </li>
 *     <li>Implement a better way to input the desired stack of pixels to go to. </li>
 *     <li>Clear up magic numbers. </li>
 *     <li>Clear up states. </li>
 *     <li>Find a better way to decide which april tag to use if there are more than one onscreen. </li>
 *     <li>Find a way to make the robot go to the right pixel stack regardless of whether it's pointing at the left or right april tags. </li>
 *     <li>Neaten up the linear opmode. </li>
 *     <li>Find out and fill in the component names. </li>
 *     <li>Test and fine tune the DISTANCE_ERROR, ANGLE_ERROR, DESIRED_DISTANCE, MAX_AUTO_SPEED and MAX_AUTO_TURN constants. </li>
 *     <li>Make the telemetry data clearer. </li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Automatic Alignment To Pixels", group = "operation-valour-test")
public class AutomaticAlignmentToPixels extends LinearOpMode {
    // TODO: fine tune these by testing
    private final double DISTANCE_ERROR = 6; // in inches
    private final double ANGLE_ERROR = 20; // in degrees

    private final double DESIRED_DISTANCE = 60; // in inches

    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_TURN = 0.3;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private MecanumDrive mecanum;

    private SensorDistanceEx distanceSensor;
    private GyroEx gyro;
    private TouchSensor touchSensor;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private final int N_PIXEL_STACKS = 6;
    private final double[] xOffsetFromAprilTags = {0, 11, 22, -22, -11, 0}; // 11 inch spacing between stacks

    private double[] closestAngle = {0, Double.POSITIVE_INFINITY}; // [angle (degrees), distance (inches)]
    private int selectedPixelStack = 0;

    private State currentState = State.IDLE;

    @Override
    public void runOpMode() {
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

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_name");
        gyro = new RevIMU(hardwareMap, "gyro_name");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor_name");

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> selectedPixelStack = (selectedPixelStack + 1) % N_PIXEL_STACKS);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> selectedPixelStack = (selectedPixelStack - 1) % N_PIXEL_STACKS);

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> currentState = State.SEARCHING_FOR_APRIL_TAGS);


        AprilTagDetection aprilTagDetection = null;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pixel stack selected", selectedPixelStack);
            telemetry.addData("Current state", currentState.toString());


            if (currentState == State.SEARCHING_FOR_APRIL_TAGS) {
                aprilTagDetection = getAprilTag();
                gyro.reset();

            }

            if (aprilTagDetection != null) {
                currentState = State.MOVING_TO_PIXEL_STACK;

                boolean hasStopped = moveToPosition(aprilTagDetection.ftcPose, xOffsetFromAprilTags[selectedPixelStack]);

                if (hasStopped) {
                    aprilTagDetection = null;
                    currentState = State.TURNING;
                }

            }

            if (currentState == State.TURNING) {
                // TODO: clear up magic numbers
                boolean hasTurned = turnToAngle(-30);

                if (hasTurned) {
                    currentState = State.SEARCHING_FOR_PIXEL_STACK;
                }
            }

            if (currentState == State.SEARCHING_FOR_PIXEL_STACK) {
                boolean hasFinished = scanForPixelStack(30, true);

                if (hasFinished) {
                    currentState = State.SHIFTING_TO_PIXEL_STACK;
                }
            }

            if (currentState == State.SHIFTING_TO_PIXEL_STACK) {
                mecanum.driveRobotCentric(0, MAX_AUTO_SPEED / 3, 0);

                if (distanceSensor.getDistance(DistanceUnit.INCH) > closestAngle[1]) { // if the robot has driven off at the wrong angle
                    currentState = State.TURNING;

                } else if (touchSensor.isPressed()) { // if the robot has reached the pixel stack
                    currentState = State.PICKING_UP_PIXEL;
                }
            }

            sleep(10);
        }

        visionPortal.close();
    }

    // gets the april tag that the robot should use for its pathfinding
    // if there are more than one april tags on the screen, then it will chose the first one returned
    @Nullable
    private AprilTagDetection getAprilTag() {
        // get the detections and filter out the ones without metadata
        List<AprilTagDetection> aprilTagDetections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .collect(Collectors.toList());

        if (aprilTagDetections.size() == 0) {
            telemetry.addLine("\tNo april tags detected");
            return null;
        } else {
            telemetry.addLine("\t" + aprilTagDetections.size() + "april tags detected");
            return aprilTagDetections.get(0); // TODO: if multiple april tags is a problem, replace this with something more rigorous
        }
    }

    // attempts to move robot to april tag, and returns if the robot has stopped
    private boolean moveToPosition(@NonNull AprilTagPoseFtc anchor, double xOffset) {
        // calculate how far the robot has to turn/move
        // idk if I have to use anchor.y or anchor.range here, but am using anchor.y for now
        double headingError = Math.atan2(anchor.y, anchor.x + xOffset) * 180 / Math.PI;
        double distanceError = distanceSensor.getDistance(DistanceUnit.INCH);

        double currentHeading = gyro.getHeading();
        // set the angle to be from -179 to 180 instead of 0 to 359
        currentHeading = currentHeading > 180 ? currentHeading - 360 : currentHeading;

        if (!(Math.abs(headingError - currentHeading) <= ANGLE_ERROR)) {
            telemetry.addLine("\tTurning to correct angle");

            turnToAngle(headingError);

        } else if (!(Math.abs(distanceError - DESIRED_DISTANCE) <= DISTANCE_ERROR)) {
            telemetry.addLine("\tMoving to correct distance");

            driveToDistance(distanceError, DistanceUnit.INCH);

        } else {
            return true;
        }

        return false;
    }

    private boolean scanForPixelStack(double searchAngle, boolean isClockwise) {
        if (distanceSensor.getDistance(DistanceUnit.INCH) < closestAngle[1]) {
            closestAngle[0] = gyro.getHeading();
            closestAngle[1] = distanceSensor.getDistance(DistanceUnit.INCH);
        }

        return turnToAngle(isClockwise ? searchAngle : -searchAngle);
    }

    // turns robot to angle
    // note that angle should be -179 to 180 rather than 0 to 360
    // returns whether it's done turning

    private boolean turnToAngle(double angle) {
        double currentHeading = gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();

        if (Math.abs(currentHeading - angle) <= ANGLE_ERROR) {
            return true;

        } else {
            // TODO: clear up magic numbers
            mecanum.driveRobotCentric(0, 0,
                    Range.clip(MAX_AUTO_TURN * (currentHeading - angle) / 45, -MAX_AUTO_TURN, MAX_AUTO_TURN));

            return false;
        }
    }

    // drives robot forwards to distance
    // returns whether it's done driving
    private boolean driveToDistance(double distance, DistanceUnit distanceUnit) {
        double currentDistance = distanceSensor.getDistance(distanceUnit);

        if (Math.abs(currentDistance - distance) <= DISTANCE_ERROR) {
            return true;

        } else {
            // TODO: clear up magic numbers
            mecanum.driveRobotCentric(0,
                    Range.clip(MAX_AUTO_SPEED * (distance - currentDistance) /distanceUnit.fromInches(6),
                            -MAX_AUTO_SPEED, MAX_AUTO_SPEED), 0);
            return false;
        }
    }

    private enum State {
        IDLE("Idle"),
        SEARCHING_FOR_APRIL_TAGS("Searching for april tags"),
        MOVING_TO_PIXEL_STACK("Moving to stack"),
        TURNING("Turning"),
        SEARCHING_FOR_PIXEL_STACK("Searching for stack"),
        SHIFTING_TO_PIXEL_STACK("Shifting to  stack"),
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