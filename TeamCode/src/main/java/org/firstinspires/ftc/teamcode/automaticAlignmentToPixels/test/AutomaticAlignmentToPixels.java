package org.firstinspires.ftc.teamcode.automaticAlignmentToPixels.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Prototype for aligning robot to a stack of pixels (<b>untested and in developement</b>). <br />
 * It's meant to go to a specified pixel stack. <br /><br />
 * <p>
 * Controls:
 * <ul>
 *     <li>DPAD left/right - Select pixel stack</li>
 *     <li>Cross - Start/stop moving</li>
 * </ul>
 */

@Disabled
@TeleOp(name = "Automatic Alignment To Pixels", group = "operation-valour-test")
public class AutomaticAlignmentToPixels extends OpMode {
    // TODO: fine tune these by testing
    public final double DISTANCE_ERROR = 6; // in inches
    public final double ANGLE_ERROR = 20; // in degrees

    public final double ARM_ERROR = 15;
    public final double LINEAR_SLIDE_ERROR = 15;

    public final double MOVING_DESIRED_DISTANCE = 18; // in inches
    public final double SHIFTING_DESIRED_DISTANCE = 6; // in inches
    public final double FINISHED_DESIRED_DISTANCE = 12; // in inches

    public final double MAX_AUTO_SPEED = 0.5;
    public final double MAX_AUTO_TURN = 0.3;
    public final double MAX_LINEAR_SLIDE_SPEED = 0.5;

    public final int ARM_LIFTED_POSITION = 0;
    public final int ARM_NOT_LIFTED_POSITION = 2000;

    public final int LINEAR_SLIDE_DOWN_POS = 0;
    public final int LINEAR_SLIDE_UP_POS = 2000;

    public final double GRABBER_CLOSED_POS = 0;
    public final double GRABBER_OPEN_POS = 90;

    public final double GRABBER_TILTED_DOWN_POS = 0;
    public final double GRABBER_TILTED_UP_POS = 90;

    public final int INITIAL_ARM_POSITION_COUNTS = 200;
    public final int INITIAL_LINEAR_SLIDE_POSITION_COUNTS = 200;

    public final double SCANNING_FOR_PIXEL_STACK_ANGLE = 30; // degrees

    public final double AUTO_DRIVING_DIVISOR = 6;
    public final double AUTO_ANGLE_DIVISOR = 45;

    public final int N_PIXEL_STACKS = 6;
    public final double[] xOffsetFromAprilTags = {6, 18, 30, -30, -18, 6}; // 12 inch spacing between stacks

    // TODO: check these are the right way round
    public final int LEFT_APRIL_TAG_ID = 7;
    public final int RIGHT_APRIL_TAG_ID = 10;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // TODO: tune these
    private PIDFController armPidf;
    private PIDFController linearSlidePidf;

    public MecanumDrive mecanum;

    private MotorEx linearSlideMotor;
    private MotorEx armMotor;
    private ServoEx grabberServo;
    private ServoEx grabberTiltServo;

    private SensorDistanceEx distanceSensor;
    private GyroEx gyro;
    private TouchSensor touchSensor;

    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private double[] closestAngle = {0, Double.POSITIVE_INFINITY}; // [angle (degrees), distance (inches)]

    private State currentState = State.IDLE;

    private AprilTagDetection aprilTagDetection;

    private int pixelStackIndex = 0;

    @Override
    public void init() {
        // TODO: fill in component names

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "front_left"),
                new Motor(hardwareMap, "front_right"),
                new Motor(hardwareMap, "back_left"),
                new Motor(hardwareMap, "back_right")
        );

        armMotor = new MotorEx(hardwareMap, "arm_name");
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionTolerance(ARM_ERROR);

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_name");
        linearSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionTolerance(LINEAR_SLIDE_ERROR);

        armPidf = new PIDFController(0, 0, 0, 0);
        linearSlidePidf = new PIDFController(0, 0, 0, 0);

        grabberServo = new SimpleServo(hardwareMap, "grabber_servo_name", GRABBER_CLOSED_POS, GRABBER_OPEN_POS);
        grabberTiltServo = new SimpleServo(hardwareMap, "grabber_tilt_servo_name", GRABBER_TILTED_DOWN_POS, GRABBER_TILTED_UP_POS);

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_name");
        gyro = new RevIMU(hardwareMap, "gyro_name");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor_name");

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> pixelStackIndex = (pixelStackIndex - 1) % N_PIXEL_STACKS);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> pixelStackIndex = (pixelStackIndex + 1) % N_PIXEL_STACKS);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(this::toggleMoving);
    }

    @Override
    public void stop() {
        stopMoving();
        visionPortal.close();
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
    }

    /**
     * If the robot isn't moving, then make it move.
     */
    public void startMoving() {
        if (currentState == State.IDLE) {
            currentState = State.SEARCHING_FOR_APRIL_TAGS;
        }
    }

    /**
     * If the robot is idle, then make it move, otherwise make it stop moving.
     */
    public void toggleMoving() {
        if (isMoving()) {
            stopMoving();
        } else {
            startMoving();
        }
    }

    /**
     * @return The current state.
     */
    public State getCurrentState() {
        return currentState;
    }

    @Override
    public void loop() {
        telemetry.addData("Current State", currentState);

        switch (currentState) {
            case SEARCHING_FOR_APRIL_TAGS:
                aprilTagDetection = getAprilTag();
                gyro.reset();

                if (aprilTagDetection != null) {
                    if (aprilTagDetection.metadata.id != LEFT_APRIL_TAG_ID && pixelStackIndex <= 2) {
                        // rotate the robot towards the correct april tag
                        mecanum.driveRobotCentric(0, 0, -MAX_AUTO_TURN);
                    } else if (aprilTagDetection.metadata.id != RIGHT_APRIL_TAG_ID && pixelStackIndex >= 3) {
                        // rotate the robot towards the correct april tag
                        mecanum.driveRobotCentric(0, 0, MAX_AUTO_TURN);
                    } else { // if the robot is facing towards the correct april tag
                        currentState = State.MOVING_TO_PIXEL_STACK;
                    }

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
                hasFinished = scanForPixelStack(SCANNING_FOR_PIXEL_STACK_ANGLE * 2);

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
                    setArmTargetPosition(ARM_LIFTED_POSITION);
                    armMotor.set(armPidf.calculate(armMotor.getCurrentPosition()));

                    setLinearSlideTargetPosition(LINEAR_SLIDE_UP_POS);
                    linearSlideMotor.set(linearSlidePidf.calculate(linearSlideMotor.getCurrentPosition()));
                }
                grabberServo.turnToAngle(GRABBER_OPEN_POS);
                grabberTiltServo.turnToAngle(GRABBER_TILTED_DOWN_POS);

                if (linearSlideMotor.atTargetPosition() && armMotor.atTargetPosition()) {
                    currentState = State.PICKING_UP_PIXEL;
                }

                break;

            case PICKING_UP_PIXEL:
                linearSlideMotor.setTargetPosition(LINEAR_SLIDE_DOWN_POS);
                if (!touchSensor.isPressed()) {
                    linearSlideMotor.set(MAX_LINEAR_SLIDE_SPEED / 2);
                } else {
                    grabberServo.turnToAngle(GRABBER_CLOSED_POS);
                    currentState = State.MOVING_BACK_FROM_PIXELS;
                }

                break;

            case MOVING_BACK_FROM_PIXELS:
                hasFinished = driveToDistance(FINISHED_DESIRED_DISTANCE, DistanceUnit.INCH);

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
     * Attempts to move robot to april tag.
     *
     * @param anchor  The april tag that the robot must move to.
     * @param xOffset The horizontal offset from the april tag, in inches.
     *                For example, if you want to move the robot half a foot right of an april tag, then pass in 6.
     * @return If it has finished moving the robot.
     */
    private boolean moveToPosition(@NonNull AprilTagPoseFtc anchor, double xOffset) {
        // calculate how far the robot has to turn/move
        // idk if I have to use anchor.y or anchor.range here, but am using anchor.y for now
        double headingError = Math.toDegrees(Math.atan2(anchor.y, anchor.x + xOffset));
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

    /**
     * Set arm PID and feedforward to desired position
     *
     * @param position The desired final position of the arm
     */
    private void setArmTargetPosition(int position) {
        armPidf.setSetPoint(position - INITIAL_ARM_POSITION_COUNTS);
    }

    /**
     * Set arm PID and feedforward to desired position
     *
     * @param position The desired final position of the arm
     */
    private void setLinearSlideTargetPosition(int position) {
        linearSlidePidf.setSetPoint(position - INITIAL_LINEAR_SLIDE_POSITION_COUNTS);
    }

    /**
     * Turns the robot clockwise while logging the closest angle in closestAngle.
     *
     * @return If it has finished scanning.
     */
    // TODO: find a better way to do this
    private boolean scanForPixelStack(double searchAngle) {
        if (distanceSensor.getDistance(DistanceUnit.INCH) < closestAngle[1]) {
            closestAngle[0] = gyro.getHeading();
            closestAngle[1] = distanceSensor.getDistance(DistanceUnit.INCH);
        }

        return turnToAngle(searchAngle);
    }


    /**
     * Turns robot to angle.
     * Note that angle should be -179 to 180 rather than 0 to 360.
     *
     * @return whether it's done turning
     */

    private boolean turnToAngle(double angle) {
        double currentHeading = gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();

        if (Math.abs(currentHeading - angle) <= ANGLE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0, 0,
                    Range.clip(MAX_AUTO_TURN * (currentHeading - angle) / AUTO_ANGLE_DIVISOR, -MAX_AUTO_TURN, MAX_AUTO_TURN));

            return false;
        }
    }

    /**
     * Drives the robot to the specified distance.
     *
     * @param distance     The distance that the robot must drive to.
     * @param distanceUnit The unit that <i>distance</i> is in.
     * @return If it has finished moving the robot.
     */
    private boolean driveToDistance(double distance, DistanceUnit distanceUnit) {
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

    /**
     * The state that the robot is currently in.
     */
    public enum State {
        IDLE("Idle"),
        SEARCHING_FOR_APRIL_TAGS("Searching for april tags"),
        MOVING_TO_PIXEL_STACK("Moving to stack"),
        TURNING("Turning"),
        SEARCHING_FOR_PIXEL_STACK("Searching for stack"),
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