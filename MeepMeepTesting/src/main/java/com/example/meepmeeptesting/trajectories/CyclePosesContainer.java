package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * A class to house poses for the backdrop cycle positions, which only have a single version per team colour.
 */
public class CyclePosesContainer {
    public static final CyclePosesContainer RED_CYCLE_POSES = new CyclePosesContainer(
            new Pose2d(50, -30, Math.toRadians(180)),
            new Pose2d(50, -40, Math.toRadians(180)),
            new Pose2d(50, -35, Math.toRadians(180)),

            new Pose2d(-60, -36, Math.toRadians(180)),
            new Pose2d(60, -60, Math.toRadians(180))
    );

    public static final CyclePosesContainer BLUE_CYCLE_POSES = new CyclePosesContainer(
            new Pose2d(50, 30, Math.toRadians(180)),
            new Pose2d(50, 40, Math.toRadians(180)),
            new Pose2d(50, 35, Math.toRadians(180)),

            new Pose2d(-60, 36, Math.toRadians(180)),
            new Pose2d(60, 60, Math.toRadians(180))
    );

    public final Pose2d leftBackdropPose;
    public final Pose2d rightBackdropPose;
    public final Pose2d centerBackdropPose;
    public final Pose2d pixelStackPose;
    public final Pose2d parkPose;

    public CyclePosesContainer(Pose2d leftBackdropPose,
                               Pose2d rightBackdropPose,
                               Pose2d centerBackdropPose,
                               Pose2d pixelStackPose,
                               Pose2d parkPose) {
        this.leftBackdropPose = leftBackdropPose;
        this.rightBackdropPose = rightBackdropPose;
        this.centerBackdropPose = centerBackdropPose;
        this.pixelStackPose = pixelStackPose;
        this.parkPose = parkPose;
    }
}
