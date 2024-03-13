package org.firstinspires.ftc.teamcode.opMode.test;

import static org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor.PropPositions.LEFT;
import static org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor.PropPositions.RIGHT;
import static org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor.PropPositions.UNFOUND;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

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
public abstract class BasicAutonomousGeneric extends OpMode {
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    // TODO: fine tune these values
    public static final double ANGLE_ERROR = 10;

    public static final double DRIVING_TO_BACKDROP_DIST = 6;

    public static final double ANGLE_DIVISOR = 90;

    public static final double PARK_TIME = 1;
    public static final double PURPLE_INITIAL_FORWARD_SECONDS = 0.4;
    public static final double PURPLE_MIDDLE_FORWARDS_SECONDS = 0.2;

    public static final Size CAMERA_SIZE = new Size(640, 480);

    public final double CV_MIN_AREA = 100; // the minimum area for the detection to consider for your prop

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private MecanumDrive mecanum;

    private ArmComponent arm;
    private OuttakeComponent outtake;

    private SensorDistanceEx distanceSensor;
    private IMU imu;

    private Runnable currentState = this::driveToSpikeMarks;

    // fill these in in the color specific opmode
    protected TeamColor teamColor = null;
    protected double backdropTurningAngle = 0;

    protected Scalar cvLower;
    protected Scalar cvUpper;

    protected int[] aprilTagIds; // left, middle, right

    private ColourMassDetectionProcessor.PropPositions recordedPropPosition;

    private ElapsedTime time;

    // TODO: fill in constants

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // hardware

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        mecanum = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);

//        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"), lynxModule.getInputVoltage(VoltageUnit.VOLTS));
//        outtake = new OuttakeComponent(new SimpleServo(hardwareMap, "outtake_servo", 0, 360));

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor_front");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        // vision stuff

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                cvLower,
                cvUpper,
                () -> CV_MIN_AREA, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(CAMERA_SIZE)
                .addProcessors(colourMassDetectionProcessor, aprilTag)
                .build();
    }

    @Override
    public void start() {
        imu.resetYaw();

        // gets the recorded prop position
        recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        time = new ElapsedTime();
    }

    @Override
    public void loop() {
//        arm.read();

        currentState.run();

        telemetry.addData("Detected prop placement", recordedPropPosition.toString());
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }

    private void driveToSpikeMarks() {
        telemetry.addLine("Driving to spike marks.");

        if (time.seconds() < PURPLE_INITIAL_FORWARD_SECONDS) {
            mecanum.driveRobotCentric(0, 0.8, 0);
        } else {
            currentState = this::movingForPurplePixelPlacement;
            time.reset();
        }
    }

    private void movingForPurplePixelPlacement() {
        telemetry.addLine("Moving to place purple pixel.");

        boolean isDone;

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                isDone = turnToAngle(90);
                if (isDone) {
//                    arm.setState(ArmComponent.State.GROUND);
                    currentState = this::placePurplePixel;
                }
                break;
            case MIDDLE:
                if (time.seconds() < PURPLE_MIDDLE_FORWARDS_SECONDS) {
                    mecanum.driveRobotCentric(0, 0.8, 0);
                } else {
                    isDone = turnToAngle(180);
                    if (isDone) {
//                      arm.setState(ArmComponent.State.GROUND);
                        currentState = this::placePurplePixel;
                    }
                }
                break;
            case RIGHT:
                isDone = turnToAngle(-90);

                if (isDone) {
//                    arm.setState(ArmComponent.State.GROUND);
                    currentState = this::placePurplePixel;
                }
                break;
        }
    }

    private void placePurplePixel() {
        telemetry.addLine("Placing purple pixel");

//        outtake.releasePixel();

        currentState = this::turnToBackdrop;
    }

    private void turnToBackdrop() {
        telemetry.addLine("Turning to backdrop. ");

        boolean isDone = turnToAngle(backdropTurningAngle);

        if (isDone) {
            currentState = this::driveToBackdrop;
//            outtake.releasePixel();
        }
    }

    private void driveToBackdrop() {
        double strafeAmount;

        int aprilTagIdsIndex;
        if (recordedPropPosition == LEFT) {
            aprilTagIdsIndex = 0;
        } else if (recordedPropPosition == RIGHT) {
            aprilTagIdsIndex = 2;
        }
        else {
            aprilTagIdsIndex = 1;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections().stream()
                .filter((x) -> x.id == aprilTagIds[aprilTagIdsIndex] && x.metadata != null)
                .collect(Collectors.toList());

        if (detections.isEmpty()) {
            strafeAmount = 0;

        } else {
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose.x < 2) {
                strafeAmount = 0;
            } else if (detection.ftcPose.x > 0) {
                strafeAmount = -0.25;
            } else {
                strafeAmount = 0.25;
            }
        }

        mecanum.driveRobotCentric(strafeAmount, 0.5, 0);

        if (distanceSensor.getDistance(DistanceUnit.INCH) < DRIVING_TO_BACKDROP_DIST) {
            currentState = this::turnToPlaceYellowPixel;
        }
    }

    private void turnToPlaceYellowPixel() {
        double turningAngle = backdropTurningAngle > 0 ? backdropTurningAngle - 180 : backdropTurningAngle + 180;
        boolean isDone = turnToAngle(turningAngle);

        if (isDone) {
            currentState = this::placeYellowPixel;
        }
    }

    private void placeYellowPixel() {
        telemetry.addLine("Placing yellow pixel. ");

//        outtake.releasePixel();

        time.reset();

        currentState = this::park;
    }

    private void park() {
        telemetry.addLine("Parking. ");

        mecanum.driveRobotCentric(teamColor == TeamColor.RED ? 0.75 : -0.75, 0, 0);

        if (time.seconds() >= PARK_TIME) {
            mecanum.driveRobotCentric(0, 0, 0);
            currentState = () -> {};
        }

    }


    /**
     * Turns the robot.
     *
     * @param angle angle from -180 to 180.
     * @return whether the robot is finished turning.
     */
    private boolean turnToAngle(double angle) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Angle", angle);
        telemetry.addData("Recorded heading", heading);
        telemetry.addData("Turn amount", (heading - angle) / ANGLE_DIVISOR);

        if (Math.abs(heading - angle) <= ANGLE_ERROR) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, 0, (heading - angle) / ANGLE_DIVISOR);
            return false;
        }
    }

    public enum TeamColor {
        BLUE,
        RED
    }
}
