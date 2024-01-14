package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class containing positions of useful places on the field.
 */
public final class TrajectoryPositions {
    public static final Pose2d STARTING_POSE_RED_AUDIENCE = new Pose2d(-35, -60, 90);
    public static final Pose2d STARTING_POSE_RED_FAR = new Pose2d(10, -60, 90);

    public static final Pose2d STARTING_POSE_BLUE_AUDIENCE = new Pose2d(-35, 60, 270);
    public static final Pose2d STARTING_POSE_BLUE_FAR = new Pose2d(10, 60, 270);

    public static final Pose2d SPIKE_MARK_RED_AUDIENCE_LEFT = new Pose2d(-35, -30, 180);
    public static final Pose2d SPIKE_MARK_RED_AUDIENCE_CENTER = new Pose2d(-35, -30, 90);
    public static final Pose2d SPIKE_MARK_RED_AUDIENCE_RIGHT = new Pose2d(-35, -30, 0);

    public static final Pose2d SPIKE_MARK_RED_FAR_LEFT = new Pose2d(10, -30, 180);
    public static final Pose2d SPIKE_MARK_RED_FAR_CENTER = new Pose2d(10, -30, 90);
    public static final Pose2d SPIKE_MARK_RED_FAR_RIGHT = new Pose2d(10, -30, 0);

    public static final Pose2d SPIKE_MARK_BLUE_AUDIENCE_LEFT = new Pose2d(-35, 30, 0);
    public static final Pose2d SPIKE_MARK_BLUE_AUDIENCE_CENTER = new Pose2d(-35, 30, 270);
    public static final Pose2d SPIKE_MARK_BLUE_AUDIENCE_RIGHT = new Pose2d(-35, 30, 180);

    public static final Pose2d SPIKE_MARK_BLUE_FAR_LEFT = new Pose2d(10, 30, 0);
    public static final Pose2d SPIKE_MARK_BLUE_FAR_CENTER = new Pose2d(10, 30, 270);
    public static final Pose2d SPIKE_MARK_BLUE_FAR_RIGHT = new Pose2d(10, 30, 180);

    public static final Pose2d BACKDROP_RED_LEFT = new Pose2d(60, -30, 0);
    public static final Pose2d BACKDROP_RED_CENTER = new Pose2d(60, -35, 0);
    public static final Pose2d BACKDROP_RED_RIGHT = new Pose2d(60, -40, 0);

    public static final Pose2d BACKDROP_BLUE_LEFT = new Pose2d(60, 30, 0);
    public static final Pose2d BACKDROP_BLUE_CENTER = new Pose2d(60, 35, 0);
    public static final Pose2d BACKDROP_BLUE_RIGHT = new Pose2d(60, 40, 0);

    public static final Pose2d PIXEL_STACK_LEFT_0 = new Pose2d(-60, 35, 180);
    public static final Pose2d PIXEL_STACK_LEFT_1 = new Pose2d(-60, 23, 180);
    public static final Pose2d PIXEL_STACK_LEFT_2 = new Pose2d(-60, 10, 180);

    public static final Pose2d PIXEL_STACK_RIGHT_0 = new Pose2d(-60, -10, 180);
    public static final Pose2d PIXEL_STACK_RIGHT_1 = new Pose2d(-60, -23, 180);
    public static final Pose2d PIXEL_STACK_RIGHT_2 = new Pose2d(-60, -35, 180);
}
