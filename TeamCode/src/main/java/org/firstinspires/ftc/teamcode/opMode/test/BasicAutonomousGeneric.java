package org.firstinspires.ftc.teamcode.opMode.test;

import static org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor.PropPositions.UNFOUND;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

/**
 * Rudimentary autonomous code. <br />
 * Moves to the Team Prop, places down the purple pixel, drives to the backdrop, and places the yellow pixel. <br />
 *
 * <b>
 * Note: this code uses method calls from component classes in other branches.
 * Be sure to merge the grabber-code, linear-slide-code, and arm-code branches into the branch that this is in before running.
 * </b>
 */
public class BasicAutonomousGeneric extends OpMode {
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    // TODO: fine tune these values
    public final double ANGLE_ERROR = 20;

    public final double SHIFTING_TO_BACKDROP_DIST = 3;

    public final double ANGLE_DIVISOR = 90;

    public final double MIN_DISTANCE_FROM_OBJECT = 6;

    public final double PARKING_DIST_ERROR = 3;


    private VisionPortal visionPortal;

    // TODO: set these values
    private final Size cameraSize = new Size(640, 480);

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

    private final double CV_MIN_AREA = 100; // the minimum area for the detection to consider for your prop

    private ColourMassDetectionProcessor.PropPositions recordedPropPosition;

    private ElapsedTime time;

    // TODO: fill in constants
    private final double PURPLE_INITIAL_FORWARDS_MILLISECONDS = 0;
    private final double PURPLE_MIDDLE_FORWARDS_MILLISECONDS = 0;

    @Override
    public void init() {
        // hardware

        Motor[] motors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        mecanum = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        // invert some motors (because hardware is weird)
        for (Motor motor : new Motor[]{motors[0], motors[2], motors[3]}) {
            motor.setInverted(!motor.getInverted());
        }

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"), lynxModule.getInputVoltage(VoltageUnit.VOLTS));
        outtake = new OuttakeComponent(new SimpleServo(hardwareMap, "outtake_servo", 0, 360));

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
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

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .setCameraResolution(cameraSize)
                .enableLiveView(true)
                .addProcessor(colourMassDetectionProcessor)
                .build();
    }

    @Override
    public void start() {
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

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
        arm.read();

        currentState.run();

        telemetry.update();
    }

    private void driveToSpikeMarks() {
        telemetry.addLine("Driving to spike marks.");

        if (time.time() < PURPLE_INITIAL_FORWARDS_MILLISECONDS) {
            mecanum.driveRobotCentric(0, 0.8, 0);
        } else {
            imu.resetYaw();
            currentState = this::movingForPurplePixelPlacement;
            time.reset();
        }
    }

    private void movingForPurplePixelPlacement() {
        telemetry.addLine("Strafing to place purple pixel.");

        boolean isDone;

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                isDone = turnToAngle(-90);
                if (isDone) {
                    arm.setState(ArmComponent.State.GROUND);
                    currentState = this::placePurplePixel;
                }
                break;
            case MIDDLE:
                if (time.time() < PURPLE_MIDDLE_FORWARDS_MILLISECONDS) {
                    mecanum.driveRobotCentric(0, 0.8, 0);
                } else {
                    arm.setState(ArmComponent.State.GROUND);

                    currentState = this::placePurplePixel;
                }
                break;
            case RIGHT:
                isDone = turnToAngle(90);

                if (isDone) {
                    arm.setState(ArmComponent.State.GROUND);
                    currentState = this::placePurplePixel;
                }
                break;
        }
    }

    private void placePurplePixel() {
        telemetry.addLine("Placing purple pixel");

        outtake.releasePixel();

        currentState = this::turnToBackdrop;
        imu.resetYaw();
    }

    private void turnToBackdrop() {
        telemetry.addLine("Turning to backdrop. ");

        boolean isDone = turnToAngle(backdropTurningAngle);

        if (isDone) {
            currentState = this::placeYellowPixel;
            outtake.releasePixel();
        }
    }

    private void placeYellowPixel() {
        telemetry.addLine("Placing yellow pixel. ");

        outtake.releasePixel();

        currentState = this::strafeToPark;
    }

    private void strafeToPark() {
        telemetry.addLine("Strafing to park. ");

        mecanum.driveRobotCentric(0, teamColor == TeamColor.RED ? -0.25 : 0.25, 0);

        if (distanceSensor.getDistance(DistanceUnit.INCH) > SHIFTING_TO_BACKDROP_DIST + PARKING_DIST_ERROR) {
            currentState = this::park;
        }
    }

    private void park() {
        telemetry.addLine("Parking. ");

        if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
            mecanum.driveRobotCentric(0, 0.25, 0);
        } else {
            currentState = () -> {
            };
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

        if (Math.abs(angle - heading) <= ANGLE_ERROR) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, 0, -heading / ANGLE_DIVISOR);
            return false;
        }
    }

    public enum TeamColor {
        BLUE,
        RED
    }
}