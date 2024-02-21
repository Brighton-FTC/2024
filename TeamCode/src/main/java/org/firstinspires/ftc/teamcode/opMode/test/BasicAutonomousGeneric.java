package org.firstinspires.ftc.teamcode.opMode.test;

import static org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor.PropPositions.UNFOUND;

import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.AutomaticPixelPlacement;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.components.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.Comparator;
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
public class BasicAutonomousGeneric extends OpMode {
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    // TODO: fine tune these values
    public final double ANGLE_ERROR = 20;
    public final int VISION_ERROR = 20;

    public final double SHIFTING_TO_BACKDROP_DIST = 3;

    public final double ANGLE_DIVISOR = 90;

    public final double MIN_DISTANCE_FROM_OBJECT = 6;

    public final double PARKING_DIST_ERROR = 3;

    public final double CAMERA_Y_POS = 2.0 / 3.0;

    protected VisionPortal visionPortal;

    protected TfodProcessor tfod;
    protected AprilTagProcessor aprilTag;

    // TODO: set these values
    protected Size cameraSize = new Size(640, 480);

    protected MecanumDrive mecanum;

    protected ArmComponent arm;
    protected LinearSlideComponent linearSlide;
    protected GrabberComponent grabber;

    protected AutomaticPixelPlacement automaticPixelPlacement;

    protected SensorDistanceEx distanceSensor;
    protected IMU imu;

    protected Runnable currentState = this::driveToSpikeMarks;

    protected TeamColor teamColor = null; // fill this in in the color specific opmode
    protected double backdropTurningAngle = 0; // fill this in as well

    protected ColourMassDetectionProcessor.PropPositions recordedPropPosition;

    protected ElapsedTime time;

    // TODO: fill in constants
    protected double YELLOW_INITIAL_FORWARDS_MILLISECONDS = 0;
    protected double YELLOW_MIDDLE_FORWARDS_MILLISECONDS = 0;
    protected double YELLOW_STRAFE_MILLISECONDS = 0;

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

        // invert motors (because of hardware)
        for (Motor motor : motors) {
            motor.setInverted(!motor.getInverted());
        }

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        linearSlide = new LinearSlideComponent(new MotorEx(hardwareMap, "linear_slide_motor"), arm);
        grabber = new GrabberComponent(new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360),
                new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360));

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

        automaticPixelPlacement = new AutomaticPixelPlacement(arm, linearSlide, grabber, mecanum, aprilTag, imu);

        // vision stuff
        aprilTag = new AprilTagProcessor.Builder().build();


        Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam_name"))
                .setCameraResolution(cameraSize)
                .enableLiveView(true)
                .addProcessors(tfod, colourMassDetectionProcessor)
                .build();
    }

    /**
     * User-defined start method
     * <p>
     * This method will be called once, when the play button is pressed.
     * <p>
     * This method is optional. By default, this method takes no action.
     * <p>
     * Example usage: Starting another thread.
     */
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
        linearSlide.read();

        currentState.run();

        telemetry.update();
    }

    protected void driveToSpikeMarks() {
        telemetry.addLine("Driving to spike marks.");

        if (time.time() < YELLOW_INITIAL_FORWARDS_MILLISECONDS){
            mecanum.driveRobotCentric(0, 0.8, 0);
        }
        else {
            currentState = this::strafeForPurplePixelPlacement;
        }
    }

    protected void strafeForPurplePixelPlacement() {
        telemetry.addLine("Strafing to place purple pixel.");

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                // code to do if we saw the prop on the left
                if (time.time() < YELLOW_STRAFE_MILLISECONDS){
                    mecanum.driveRobotCentric(-0.8, 0, 0);
                }
                else {
                    arm.lower();
                    linearSlide.lower();
                    currentState = this::placePurplePixel;
                }
                break;
            case MIDDLE:
                // code to do if we saw the prop on the middle
                if (time.time() < YELLOW_MIDDLE_FORWARDS_MILLISECONDS){
                    mecanum.driveRobotCentric(0, 0.8, 0);
                }
                else {
                    arm.lower();
                    linearSlide.lower();
                    currentState = this::placePurplePixel;
                }
                break;
            case RIGHT:
                // code to do if we saw the prop on the right
                if (time.time() < YELLOW_STRAFE_MILLISECONDS){
                    mecanum.driveRobotCentric(0.8, 0, 0);
                }
                else {
                    arm.lower();
                    linearSlide.lower();
                    currentState = this::placePurplePixel;
                }
                break;
        }
    }

    protected void placePurplePixel() {
        telemetry.addLine("Placing purple pixel");

        automaticPixelPlacement.placeOnGround();

        if (!automaticPixelPlacement.isPlacingPixel()) {
            currentState = this::turnToBackdrop;
            imu.resetYaw();
        }
    }

    protected void turnToBackdrop() {
        telemetry.addLine("Turning to backdrop. ");

        boolean isDone = turnToAngle(backdropTurningAngle);

        if (isDone) {
            currentState = this::placeYellowPixel;
            automaticPixelPlacement.placeOnBackdrop();
        }
    }

    protected void placeYellowPixel() {
        telemetry.addLine("Placing yellow pixel. ");

        automaticPixelPlacement.run();

        if (!automaticPixelPlacement.isPlacingPixel()) {
            currentState = this::strafeToPark;
        }
    }

    protected void strafeToPark() {
        telemetry.addLine("Strafing to park. ");

        mecanum.driveRobotCentric(0, teamColor == TeamColor.RED ? 0.5 : -0.5, 0);

        if (distanceSensor.getDistance(DistanceUnit.INCH) > SHIFTING_TO_BACKDROP_DIST + PARKING_DIST_ERROR) {
            currentState = this::park;
        }
    }

    protected void park() {
        telemetry.addLine("Parking. ");

        if (distanceSensor.getDistance(DistanceUnit.INCH) > MIN_DISTANCE_FROM_OBJECT) {
            mecanum.driveRobotCentric(0, 0.5, 0);
        } else {
            currentState = () -> {};
        }
    }


    /**
     * Turns the robot.
     *
     * @param angle angle from -180 to 180.
     * @return whether the robot is finished turning.
     */
    protected boolean turnToAngle(double angle) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (Math.abs(angle - heading) <= ANGLE_ERROR) {
            return true;
        } else {
            mecanum.driveRobotCentric(0, 0, heading / ANGLE_DIVISOR);
            return false;
        }
    }

    public enum TeamColor {
        BLUE,
        RED
    }
}