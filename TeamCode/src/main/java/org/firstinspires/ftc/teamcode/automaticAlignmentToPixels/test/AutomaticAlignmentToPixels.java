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
 * Prototype for aligning robot to a stack of pixels (<b>untested and in development</b>). <br />
 * It's meant to go to a specified pixel stack. <br />
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

    public final double MAX_AUTO_SPEED = 0.5;
    public final double MAX_AUTO_TURN = 0.3;
    public final double MAX_LINEAR_SLIDE_SPEED = 0.5;

    public final int ARM_LIFTED_POSITION = 0;
    public final int ARM_NOT_LIFTED_POSITION = 2000;

    public final int LINEAR_SLIDE_DOWN_POSITION = 0;
    public final int LINEAR_SLIDE_UP_POSITION = 2000;

    public final double GRABBER_CLOSED_POSITION = 0;
    public final double GRABBER_OPEN_POSITION = 90;

    public final double GRABBER_TILTED_DOWN_POSITION = 0;
    public final double GRABBER_TILTED_UP_POSITION = 90;

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

        // TODO: tune these
        armPidf = new PIDFController(0, 0, 0, 0);
        linearSlidePidf = new PIDFController(0, 0, 0, 0);

        grabberServo = new SimpleServo(hardwareMap, "grabber_servo_name", GRABBER_CLOSED_POSITION, GRABBER_OPEN_POSITION);
        grabberTiltServo = new SimpleServo(hardwareMap, "grabber_tilt_servo_name", GRABBER_TILTED_DOWN_POSITION, GRABBER_TILTED_UP_POSITION);

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

                    setLinearSlideTargetPosition(LINEAR_SLIDE_UP_POSITION);
                    linearSlideMotor.set(linearSlidePidf.calculate(linearSlideMotor.getCurrentPosition()));
                }
                grabberServo.turnToAngle(GRABBER_OPEN_POSITION);
                grabberTiltServo.turnToAngle(GRABBER_TILTED_DOWN_POSITION);

                if (linearSlideMotor.atTargetPosition() && armMotor.atTargetPosition()) {
                    currentState = State.PICKING_UP_PIXEL;
                }

                break;

            case PICKING_UP_PIXEL:
                linearSlideMotor.setTargetPosition(LINEAR_SLIDE_DOWN_POSITION);
                if (!touchSensor.isPressed()) {
                    linearSlideMotor.set(MAX_LINEAR_SLIDE_SPEED / 2);
                } else {
                    grabberServo.turnToAngle(GRABBER_CLOSED_POSITION);
                    currentState = State.IDLE;
                }
        }
    }

    /** Gets the April Tag that the robot should use for its pathfinding.
     * Rejects April Tags that aren't the two specific tags used.
     * If there is more than one April Tag on the screen, then it will choose the first one returned.
     *
     * @return null if there are no April Tags with the correct metadata, the first April Tag with the correct metadata returned otherwise
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
     * Moves the robot to the pixel stack using a April Tag.
     *
     * @param anchor The April Tag detected - either the left or the right one
     * @param xOffset a offset to the heading depending on which pixel stack the robot is going to
     * @return whether the robot has finished moving as a boolean
     */
    private boolean moveToPosition(@NonNull AprilTagPoseFtc anchor, double xOffset) {
        // calculate how far the robot has to turn/move
        // idk if I have to use anchor.y or anchor.range here, but am using anchor.y for now
        // TODO: figure out if this is wrong and how to fix if so
        double headingError = Math.toDegrees(Math.atan2(anchor.y, anchor.x + xOffset));
        double distanceError = distanceSensor.getDistance(DistanceUnit.INCH);

        double currentHeading = angleConvert(gyro.getHeading());

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
     * Converts angle from 0 to 360 into -180 to 180
     * @param currentHeading angle from 0 to 360 you want to convert
     * @return angle from -180 to 180
     */
    private double angleConvert(double currentHeading) {
        return currentHeading > 180 ? currentHeading - 360 : currentHeading;
    }

    /**
     * Set linear slide to desired position
     *
     * @param position The desired final position of the linear slide
     */
    private void setLinearSlideTargetPosition(int position) {
        linearSlidePidf.setSetPoint(position - INITIAL_LINEAR_SLIDE_POSITION_COUNTS);
    }

    // TODO: find a better way to do this
    /** turns clockwise while logging the closest angle in closestAngle
     * @return if robot has turned to angle yet
     */
    private boolean scanForPixelStack(double searchAngle) {
        if (distanceSensor.getDistance(DistanceUnit.INCH) < closestAngle[1]) {
            closestAngle[0] = gyro.getHeading();
            closestAngle[1] = distanceSensor.getDistance(DistanceUnit.INCH);
        }

        return turnToAngle(searchAngle);
    }

    /** Turns robot to angle.
     *
     * @param angle angle required in degrees, 0 to 360
     * @return whether it's done turning as a boolean
     */

    private boolean turnToAngle(double angle) {
        double currentHeading = angleConvert(gyro.getHeading());

        if (Math.abs(currentHeading - angle) <= ANGLE_ERROR) {
            return true;

        } else {
            mecanum.driveRobotCentric(0, 0,
                    Range.clip(MAX_AUTO_TURN * (currentHeading - angle) / AUTO_ANGLE_DIVISOR, -MAX_AUTO_TURN, MAX_AUTO_TURN));

            return false;
        }
    }

    /**
     * Drives robot forward until the distance sensor has a reading closer than the distance passed in
     *
     * @param distance the distance (detected by the distance sensor) that the method will stop moving at
     * @param distanceUnit the unit that distance is measured in
     * @return whether the distance has been reached as a boolean
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
     * Contains the possible states of the robot.
     */
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