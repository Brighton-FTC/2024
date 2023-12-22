package org.firstinspires.ftc.teamcode.components.test;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Component class for automatic pixel placement. <br />
 * To place a pixel, call {@link #placeOnBackdrop()} or {@link #placeOnGround()} once, and then call {@link #run()} continuously.
 */
public class AutomaticPixelPlacement {
    public static final double DISTANCE_FROM_APRIL_TAG = 6;

    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;

    public static final double DRIVE_DIVISOR = 6;
    public static final double STRAFE_DIVISOR = 8;
    public static final double TURN_DIVISOR = 60;

    public static final double DISTANCE_ERROR = 3;
    public static final double TURN_ERROR = 20;

    private final ArmComponent arm;
    private final LinearSlideComponent linearSlide;
    private final GrabberComponent grabber;
    private final MecanumDrive mecanum;

    private final AprilTagProcessor aprilTag;

    private final IMU imu;

    private Runnable currentState = () -> {
    };

    private boolean isDone = true;

    /**
     * Component class for automatic pixel placement.
     *
     * @param arm         The arm component.
     * @param linearSlide The linear slide component.
     * @param grabber     The grabber component.
     * @param mecanum     The mecanum drive.
     * @param aprilTag    The april tag processor.
     * @param imu         The imu.
     */
    public AutomaticPixelPlacement(ArmComponent arm,
                                   LinearSlideComponent linearSlide,
                                   GrabberComponent grabber,
                                   MecanumDrive mecanum,
                                   AprilTagProcessor aprilTag,
                                   IMU imu) {
        this.arm = arm;
        this.linearSlide = linearSlide;
        this.grabber = grabber;
        this.mecanum = mecanum;

        this.imu = imu;

        this.aprilTag = aprilTag;
    }

    /**
     * Setup the opmode to place a pixel on a backdrop. <br />
     * You need to call {@link #run()} continuously for the robot to actually do stuff.
     */
    public void placeOnBackdrop() {
        isDone = false;
        currentState = this::driveToBackdrop;
    }

    /**
     * Setup the opmode to place a pixel on the ground <br />
     * You need to call {@link #run()} continuously for the robot to actually do stuff.
     */
    public void placeOnGround() {
        isDone = false;
        currentState = this::lowerArm;
    }

    /**
     * Run the current action. <br />
     * This won't do anything unless you have already called {@link #placeOnBackdrop()} or {@link #placeOnGround()}.
     */
    public void run() {
        currentState.run();
    }

    /**
     * Get if the component class is active.
     *
     * @return If a pixel is being placed.
     */
    public boolean isPlacingPixel() {
        return !isDone;
    }

    /**
     * Drive the robot to the backdrop. <br />
     * Once done, it sets currentState to {@link #turnToPlacePixel()}.
     */
    private void driveToBackdrop() {
        List<AprilTagDetection> detections = getAprilTagDetections();

        if (detections.size() > 0) {
            boolean isDone = driveToAprilTag(detections.get(0), DISTANCE_FROM_APRIL_TAG);
            if (isDone) {
                imu.resetYaw();
                currentState = this::turnToPlacePixel;
            }
        } else {
            mecanum.driveRobotCentric(0, DRIVE_SPEED, 0);
        }
    }

    /**
     * Turn the robot 180 degrees, so it is pointing backwards and it can place a pixel. <br />
     * Once done, it sets currentState to {@link #liftArm()}.
     */
    private void turnToPlacePixel() {
        boolean isDone = turnToAngle(180);
        if (isDone) {
            currentState = this::liftArm;
        }
    }

    /**
     * Places the pixel onto the backdrop. <br />
     * Once done, it sets currentState to {@link #openGrabber()}.
     */
    private void liftArm() {
        arm.lift();
        linearSlide.lift();

        if (!(arm.atSetPoint() && linearSlide.atSetPoint())) {
            arm.moveToSetPoint();
            linearSlide.moveToSetPoint();
        } else {
            currentState = this::openGrabber;
        }
    }

    /**
     * Places the pixel onto the ground. <br />
     * Once done, it sets currentState to {@link #openGrabber()}.
     */
    private void lowerArm() {
        arm.lower();
        linearSlide.lift();

        if (!(arm.atSetPoint() && linearSlide.atSetPoint())) {
            arm.moveToSetPoint();
            linearSlide.moveToSetPoint();
        } else {
            currentState = this::openGrabber;
        }
    }

    /**
     * Open the grabber.
     */
    private void openGrabber() {
        grabber.open();

        currentState = () -> {
        };
        isDone = true;
    }

    /**
     * Get all april tag detections with metadata.
     *
     * @return The april tag detections.
     */
    private List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections().stream()
                .filter((detection) -> detection.metadata != null)
                .sorted((Comparator.comparingDouble(detection -> detection.ftcPose.range)))
                .collect(Collectors.toList());
    }

    /**
     * Drive the robot to an april tag detection.
     *
     * @param detection The april tag detection.
     * @param distance  The distance from the april tag that the robot is driven to.
     * @return Whether it has finished or not.
     */
    private boolean driveToAprilTag(@NonNull AprilTagDetection detection, double distance) {
        if (Math.abs(detection.ftcPose.y - distance) > DISTANCE_ERROR) {
            mecanum.driveRobotCentric(detection.ftcPose.x / STRAFE_DIVISOR, detection.ftcPose.y / DRIVE_DIVISOR, detection.ftcPose.yaw / TURN_DIVISOR);
            return false;

        } else {
            return true;
        }
    }

    /**
     * Turn the robot to an angle (relative to 0 heading).
     *
     * @param angle The angle, between -180 and 180.
     * @return If it has finished or not.
     */
    private boolean turnToAngle(double angle) {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (Math.abs(angle - currentAngle) > TURN_ERROR) {
            if (angle < currentAngle) {
                mecanum.driveRobotCentric(0, 0, TURN_SPEED);
            } else {
                mecanum.driveRobotCentric(0, 0, -TURN_SPEED);
            }

            return false;
        } else {
            return true;
        }
    }
}
