package org.firstinspires.ftc.teamcode.opModes.autonomous;

import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.DELTA_F;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

public class AprilTagLocalizer implements Localizer {

    private AprilTagProcessor aprilTag;

    private IMU imu;

    private Pose2d currentPose = new Pose2d();

    public AprilTagLocalizer(AprilTagProcessor aprilTag) {
        this.aprilTag = aprilTag;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        currentPose = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .collect(Collectors.toList());

        if (detections.isEmpty()) {
            return;
        }

        AprilTagDetection aprilTag = detections.get(0);

        Vector2d cameraVector = new Vector2d(aprilTag.ftcPose.y, -aprilTag.ftcPose.x);
        VectorF tagPosition = aprilTag.metadata.fieldPosition;
        Vector2d rTag = new Vector2d(tagPosition.get(0),tagPosition.get(1));
        Vector2d returnVector = rTag.minus(DELTA_F);
        returnVector = returnVector.minus(cameraVector);
        currentPose = new Pose2d(returnVector,
                Math.toRadians(-aprilTag.ftcPose.yaw));
    }
}
