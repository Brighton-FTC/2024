package org.firstinspires.ftc.teamcode.opModes.autonomous;

import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.cameraCenterXOffsetInches;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.cameraCenterYOffsetInches;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.deltaF;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

@Disabled
@Autonomous(name = "April Tag Localization Tester", group = "test")
public class AprilTagLocalization extends OpMode {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private BCGyro gyro;

    @Override
    public void init() {
        gyro = new BCGyro(hardwareMap);
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .setCameraResolution(CAMERA_RESOLUTION)
                .build();

        gyro.init();
    }

    @Override
    public void loop() {
        Pose2d pose = getPoseFromAtag();
        if (pose != null){
            telemetry.addData("X: ", pose.getX());
            telemetry.addData("Y: ", pose.getY());
            telemetry.addData("Heading: ", pose.getHeading());
        }
        else{
            telemetry.addLine("No tags detected");
        }
        telemetry.update();
    }
    @Override
    public void stop(){
        visionPortal.close();
    }

    @Nullable
    private Pose2d getPoseFromAtag(){
        // get the detections and filter out the ones without metadata

        List<AprilTagDetection> detections = aprilTag.getDetections()
                .stream()
                .filter(detection -> detection.metadata != null)
                .collect(Collectors.toList());

//            List<AprilTagDetection> detections = findDetections();
        if (detections == null || detections.isEmpty()) {
            RobotLog.i("getPoseFromAprilTag: no detections");
            return null;
        }
        AprilTagDetection OurTag = detections.get(0);
        for (AprilTagDetection d : detections) {
            if (OurTag.ftcPose == null) {
                OurTag = d;
                continue;
            }
            if (d.ftcPose == null) { continue; }
            if (Math.abs(d.ftcPose.x) < Math.abs(OurTag.ftcPose.x)) {
                OurTag = d; }
        }
        if (OurTag.ftcPose == null) {
            RobotLog.i("getPoseFromAprilTag: no detections"); return null;
        }
        Vector2d cameraVector = new Vector2d(OurTag.ftcPose.y, -OurTag.ftcPose.x);
        VectorF tagPosition = OurTag.metadata.fieldPosition;
        Vector2d rTag = new Vector2d(tagPosition.get(0),tagPosition.get(1));
        Vector2d returnVector = rTag.minus(deltaF);
        returnVector = returnVector.minus(cameraVector);
        Pose2d returnPose = new Pose2d(returnVector,
                Math.toRadians(-OurTag.ftcPose.yaw));

        RobotLog.i("getPoseFromAprilTag: reference tag = "+OurTag.id); RobotLog.i(String.format("getPoseFromAprilTag: tag data: (%.3f, %.3f) @%.3f",OurTag.ftcPose.x, OurTag.ftcPose.y, OurTag.ftcPose.yaw));
        RobotLog.i("getPoseFromAprilTag: pose = "+returnPose.toString());
        return returnPose;


//        if (detections.size() == 0) {
//            return null;
//        }
//
//        VectorF tempVector = new VectorF(0f, 0f);
//        float vectorCount = 0;
//
//        double heading = gyro.getHeading();
//        double botHeading = -heading;
//
//        for (AprilTagDetection detection : detections) {
////            // heading is IMU heading
//            VectorF tagPosition = detection.metadata.fieldPosition; // absolute position
//            double xRelative = detection.ftcPose.x - cameraCenterXOffsetInches; // relative position (lateral)
//            double yRelative = detection.ftcPose.y - cameraCenterYOffsetInches; // relative position (forward)
//
//            double xAbsolute = xRelative * Math.cos(botHeading) + yRelative * Math.sin(botHeading);
//            double yAbsolute = xRelative * -Math.sin(botHeading) + yRelative * Math.cos(botHeading);
//
//            tempVector.add(new VectorF(
//                    (float) (tagPosition.get(0) + yAbsolute),
//                    (float) (tagPosition.get(1) - xAbsolute)
//            ));
//            vectorCount++;
//        }
//
//        tempVector.multiply(1/vectorCount);
        /*
        perpendicular offset is front/back (front is positive)
        lateral offset is left/right (right is positive)
         */
//        Pose2d pose = new Pose2d(
//                tempVector.get(0),
//                tempVector.get(1),
//                heading
//        );
//        return pose;
    }
}
