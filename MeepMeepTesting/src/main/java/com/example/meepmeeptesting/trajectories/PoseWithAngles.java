package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseWithAngles {
    private final Pose2d pose2d;
    private final double heading;
    private final double tangent;

    public PoseWithAngles(Pose2d pose2d) {
        this(pose2d, pose2d.heading.toDouble());
    }

    public PoseWithAngles(Pose2d pose2d, double heading) {
        this(pose2d, heading, pose2d.heading.toDouble());
    }

    public PoseWithAngles(Pose2d pose2d, double heading, double tangent) {
        this.pose2d = pose2d;
        this.heading = heading;
        this.tangent = tangent;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }

    public double getHeading() {
        return heading;
    }

    public double getTangent() {
        return tangent;
    }
}
