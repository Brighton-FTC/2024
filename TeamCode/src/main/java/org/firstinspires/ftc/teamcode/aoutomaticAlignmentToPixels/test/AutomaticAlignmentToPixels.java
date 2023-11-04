package org.firstinspires.ftc.teamcode.aoutomaticAlignmentToPixels.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
 *     <li>Implement a better way to input the desired stack of pixels to go to. </li>
 *     <li>See if the current code rotates the robot towards the april tag instead of towards the wall, and if so find a better way to move the robot</li>
 *     <li>Find a better way to decide which april tag to use if there are more than one onscreen. </li>
 *     <li>Find a way to make the robot go to the right pixel stack regardless of whether it's pointing at the left or right april tags. </li>
 *     <li>Find out and fill in the component names. </li>
 *     <li>Test and fine tune the DESIRED_DISTANCE, SPEED/STRAFE/TURN_GAIN and MAX_AUTO_SPEED/STRAFE/TURN constants. </li>
 *     <li>Make the telemetry data clearer. </li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Automatic Alignment To Pixels", group = "operation-valour-test")
public class AutomaticAlignmentToPixels extends LinearOpMode {
    // TODO: fine tune these by testing
    private final double DESIRED_DISTANCE = 24; // in inches

    private final double SPEED_GAIN = 0.02;
    private final double STRAFE_GAIN = 0.015;
    private final double TURN_GAIN = 0.01;

    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_STRAFE = 0.5;
    private final double MAX_AUTO_TURN = 0.3;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private MecanumDrive mecanum;

    // private SensorDistanceEx distanceSensor;
    // private GyroEx gyro;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private final int N_PIXEL_STACKS = 6;
    private final double[] xOffsetFromAprilTags = {0, 11, 22, -22, -11, 0}; // 11 inch spacing between stacks
    private int selectedPixelStack = 0;

    private boolean isLookingForAprilTags = false;

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

        // distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_name");
        // gyro = new RevIMU(hardwareMap, "gyro_name");

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> selectedPixelStack = selectedPixelStack + 1 % N_PIXEL_STACKS);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> selectedPixelStack = selectedPixelStack - 1 % N_PIXEL_STACKS);

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> isLookingForAprilTags = !isLookingForAprilTags);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pixel stack selected", selectedPixelStack);

            if (isLookingForAprilTags) {
                telemetry.addLine("== Robot is detecting april tags ==");

                AprilTagDetection aprilTagDetection = getAprilTag();

                if (aprilTagDetection != null) {
                    boolean hasStopped = moveToPosition(aprilTagDetection.ftcPose, xOffsetFromAprilTags[selectedPixelStack]);

                    if (hasStopped) {
                        isLookingForAprilTags = false;
                    }
                } else {
                    isLookingForAprilTags = false;
                }
            }
        }

        visionPortal.close();
    }

    // gets the april tag that the robot should use for its pathfinding
    // if there are more than one april tags on the screen, then it will chose the first one returned
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
        telemetry.addLine("== Robot is moving to april tag ==");

        // calculate how far the robot has to go to be in the desired position
        double  rangeError = anchor.range - DESIRED_DISTANCE;
        double  headingError = anchor.bearing;
        double  yawError = anchor.yaw + xOffset;

        // calculate how the robot should move
        // TODO: if this code makes the robot turn towards the april tag, not towards the wall, figure out a way of fixing that
        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        mecanum.driveRobotCentric(strafe, drive, turn);

        return anchor.range <= DESIRED_DISTANCE; // TODO: figure out a better way to tell when the robot has stopped
    }
}
