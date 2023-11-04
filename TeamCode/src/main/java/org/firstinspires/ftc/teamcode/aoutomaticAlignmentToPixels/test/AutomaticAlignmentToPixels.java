package org.firstinspires.ftc.teamcode.aoutomaticAlignmentToPixels.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Prototype for aligning robot to a stack of pixels (<b>untested and in developement</b>). <br />
 * It's meant to go to an april tag when an input is given, and then strafe until another input is given. <br />
 *
 * Controls:
 * <ul>
 *     <li>X - Start going to an april tag (the leftmost one if there are more than one onscreen) and then strafe right until X is pressed again. </li>
 *     <li>Y - Start going to april tag (the rightmost one if there are more than one onscreen) and then strafe left until Y pressed again. </li>
 * </ul>
 *
 * Stuff to do:
 * <ul>
 *     <li>Implement a way to input the desired stack of pixels to go to. </li>
 *     <li>Find way to make robot go directly to the specified stack of pixels. </li>
 *     <li>Get the distance from each april tag to each stack of pixels, and put it in the code to offset the robot from the april tags. </li>
 *     <li>Find a better way to decide which april tag to use if there are more than one onscreen. </li>
 *     <li>Find out and fill in the component names. </li>
 *     <li>Test and fine tune the DESIRED_DISTANCE, SPEED/STRAFE/TURN_GAIN and MAX_AUTO_SPEED/STRAFE/TURN constants. </li>
 *     <li>Make the telemetry data clearer. </li>
 *     <li>Find a way to get rid of the inches and replace them with metric units</li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Automatic Alignment To Pixels", group = "operation-valour-test")
public class AutomaticAlignmentToPixels extends OpMode {
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

    private SensorDistanceEx distanceSensor;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private boolean isLookingForAprilTags = false;
    private boolean isStrafeDirectionRight;
    private boolean isStrafing = false;

    @Override
    public void init() {
        // TODO: fill in component names

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .addProcessor(aprilTag)
                .build();

        mecanum  = new MecanumDrive(
                new Motor(hardwareMap, "front_left"),
                new Motor(hardwareMap, "front_right"),
                new Motor(hardwareMap, "back_left"),
                new Motor(hardwareMap, "back_right")
        );

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_name");

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            if (isStrafing) {
                isStrafing = false;
            } else {
                isLookingForAprilTags = !isLookingForAprilTags;
            }

            isStrafeDirectionRight = true;
        });

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            if (isStrafing) {
                isStrafing = false;
            } else {
                isLookingForAprilTags = !isLookingForAprilTags;
            }

            isStrafeDirectionRight = false;
        });
    }

    @Override
    public void loop() {
        if (isLookingForAprilTags) {
            telemetry.addLine("== Robot is detecting april tags ==");

            AprilTagDetection aprilTagDetection = getAprilTag(isStrafeDirectionRight);

            if (aprilTagDetection != null) {
                boolean hasStopped = moveToPosition(aprilTagDetection.ftcPose);

                if (hasStopped) {
                    isStrafing = true;
                    isLookingForAprilTags = false;
                }
            }
        }

        if (isStrafing) {
            if (isStrafeDirectionRight) {
                telemetry.addLine("== Robot is strafing right ==");

                mecanum.driveRobotCentric(MAX_AUTO_STRAFE, 0, 0);
            } else {
                telemetry.addLine("== Robot is strafing left ==");

                mecanum.driveRobotCentric(-MAX_AUTO_STRAFE, 0, 0);
            }
        }
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    // gets the april tag that the robot should use for its pathfinding
    // if there are more than one april tags on the screen, then it will detect the leftmost one if isStrafeDirectionRight is true, otherwise the rightmost one
    private AprilTagDetection getAprilTag(boolean isStrafeDirectionRight) {
        List<AprilTagDetection> aprilTagDetections = aprilTag.getDetections();

        if (aprilTagDetections.size() == 0) {
            telemetry.addLine("\tNo april tags detected");
            return null;
        } else if (aprilTagDetections.size() == 1) {
            telemetry.addLine("\t1 april tag detected");
            return aprilTagDetections.get(0);
        } else {
            telemetry.addLine("\t" + aprilTagDetections.size() + "april tags detected");

            // TODO: figure out if there's a better way to do this
            AprilTagDetection returnedDetection = aprilTagDetections.get(0);
            for (int i = 1; i < aprilTagDetections.size(); i++) {
                if (isStrafeDirectionRight) { // go to the left and strafe right
                    if (aprilTagDetections.get(i).ftcPose.x > returnedDetection.ftcPose.x) {
                        returnedDetection = aprilTagDetections.get(i);
                    }
                } else { // go to the right and strafe left
                    if (aprilTagDetections.get(i).ftcPose.x < returnedDetection.ftcPose.x) {
                        returnedDetection = aprilTagDetections.get(i);
                    }
                }
            }
            return returnedDetection;
        }
    }

    // attempts to move robot to april tag, and returns if the robot has stopped
    // TODO: figure out a way to make the robot go directly to the desired stack, using xOffset
    private boolean moveToPosition(AprilTagPoseFtc anchor) { //, double xOffset) {
        if (distanceSensor.getDistance(DistanceUnit.INCH) > DESIRED_DISTANCE) {
            telemetry.addLine("== Robot is moving to april tag ==");
            telemetry.addData("\tDistance to wall (meters)", distanceSensor.getDistance(DistanceUnit.METER));

            // calculate how far the robot has to go to be in the desired position
            double  rangeError = anchor.range - DESIRED_DISTANCE;
            double  headingError = anchor.bearing;
            double  yawError = anchor.yaw;

            // calculate how the robot should move
            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            mecanum.driveRobotCentric(strafe, drive, turn);

            return false;
        } else {
            return true;
        }
    }
}
