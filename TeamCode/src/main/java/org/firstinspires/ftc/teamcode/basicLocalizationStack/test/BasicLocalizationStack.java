package org.firstinspires.ftc.teamcode.basicLocalizationStack.test;

import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A very basic attempt at a localization stack (<b>untested and in development</b>). <br />
 * <p>
 * The code uses a webcam and in the future will use four distance sensors.
 */
public class BasicLocalizationStack {
    private double startAngle;

    private final SensorDistanceEx[] distanceSensors;

    private final GyroEx gyro;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private HashMap<Integer, Point> nearSideAprilTags = new HashMap<>(); // stores the ids and positions of the april tags on the field.
    private HashMap<Integer, Point> farSideAprilTags = new HashMap<>(); // stores the ids and positions of the april tags on the field.

    /**
     * A very basic attempt at a localization stack.
     *
     * @param hardwareMap The hardware map.
     * @param startAngle  The angle that the robot starts at, in degrees (facing directly towards the backdrop is 0 degrees).
     */
    public BasicLocalizationStack(HardwareMap hardwareMap, double startAngle) {
        this.startAngle = startAngle < 0 ? startAngle + 360 : startAngle;

        nearSideAprilTags.put(10, new Point(18, 72));
        nearSideAprilTags.put(7, new Point(54, 72));

        farSideAprilTags.put(1, new Point(15, 6));
        farSideAprilTags.put(2, new Point(18, 6));
        farSideAprilTags.put(3, new Point(21, 6));
        farSideAprilTags.put(4, new Point(51, 6));
        farSideAprilTags.put(5, new Point(54, 6));
        farSideAprilTags.put(6, new Point(57, 6));

        distanceSensors = new SensorDistanceEx[]{
                new SensorRevTOFDistance(hardwareMap, "front_sensor"),
                new SensorRevTOFDistance(hardwareMap, "left_sensor"),
                new SensorRevTOFDistance(hardwareMap, "right_sensor"),
                new SensorRevTOFDistance(hardwareMap, "back_sensor")
        };

        gyro = new RevIMU(hardwareMap, "gyro");

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
    }

    /**
     * Get the approximate location of the robot (in inches), relative to the top left corner of the pitch.
     *
     * @return The approximate location of the robot in inches. Can return null if there is not enough information to infer this.
     */
    @Nullable
    public Point getRobotLocation() {
        List<AprilTagDetection> aprilTagDetections = getAprilTagDetections();

        if (aprilTagDetections.size() == 0) {
            return null;
        } else {
            AprilTagDetection detection = aprilTagDetections.get(0);

            if (nearSideAprilTags.containsKey(detection.id)) {
                double x = nearSideAprilTags.get(detection.id).x + detection.ftcPose.x;
                double y = nearSideAprilTags.get(detection.id).y - detection.ftcPose.y;

                return new Point(x, y);
            } else if (farSideAprilTags.containsKey(detection.id)) {
                double x = farSideAprilTags.get(detection.id).x + detection.ftcPose.x;
                double y = farSideAprilTags.get(detection.id).y + detection.ftcPose.x;

                return new Point(x, y);
            } else {
                return null;
            }
        }
    }


    private List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections().stream()
                .filter((detection) -> detection.metadata != null)
                .collect(Collectors.toList());
    }
}
