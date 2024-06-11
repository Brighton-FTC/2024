package com.example.meepmeeptesting.trajectories;


import com.acmerobotics.roadrunner.Pose2d;

/**
 * A class to house trajectories and poses.
 */
public final class PosesContainer {
    // TODO: fill in more precisely

    /**
     * Use like STARTING_POSES[alliance index][starting pose index]
     */
    public static final Pose2d[][] STARTING_POSES = {
            {
                    new Pose2d(-35, -60, Math.toRadians(90)),
                    new Pose2d(10, -60, Math.toRadians(90))
            },
            {
                    new Pose2d(-35, 60, Math.toRadians(-90)),
                    new Pose2d(10, 60, Math.toRadians(-90))
            }
    };

    /**
     * Use like SPIKE_MARK_POSES[alliance index][starting position index][randomization index]
     */
    public static final PoseWithAngles[][][] SPIKE_MARK_POSES = {
            {
                    {
                            new PoseWithAngles(new Pose2d(-35, -32, Math.toRadians(180))),
                            new PoseWithAngles(new Pose2d(-35, -32, Math.toRadians(0))),
                            new PoseWithAngles(new Pose2d(-35, -32, Math.toRadians(90)))
                    },
                    {

                            new PoseWithAngles(new Pose2d(10, -30, Math.toRadians(180))),
                            new PoseWithAngles(new Pose2d(10, -30, Math.toRadians(0))),
                            new PoseWithAngles(new Pose2d(10, -30, Math.toRadians(90))),
                    }
            },
            {
                    {

                            new PoseWithAngles(new Pose2d(-35, 30, Math.toRadians(0))),
                            new PoseWithAngles(new Pose2d(-35, 30, Math.toRadians(180))),
                            new PoseWithAngles(new Pose2d(-35, 30, Math.toRadians(-90)))
                    },
                    {
                            new PoseWithAngles(new Pose2d(10, 30, Math.toRadians(0))),
                            new PoseWithAngles(new Pose2d(10, 30, Math.toRadians(180))),
                            new PoseWithAngles(new Pose2d(10, 30, Math.toRadians(-90))),
                    }
            }
    };

    /**
     * Use like BACKDROP_POSES[alliance index][randomization index]
     */
    public static final PoseWithAngles[][] BACKDROP_POSES = {
            {
                    new PoseWithAngles(new Pose2d(60, -30, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(-20)),
                    new PoseWithAngles(new Pose2d(60, -40, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(-20)),
                    new PoseWithAngles(new Pose2d(60, -35, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(-20))
            },
            {
                    new PoseWithAngles(new Pose2d(60, 30, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(20)),
                    new PoseWithAngles(new Pose2d(60, 40, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(20)),
                    new PoseWithAngles(new Pose2d(60, 35, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(20))
            }
    };

    /**
     * Use like PIXEL_STACK_POSES[alliance index]
     */
    public static final PoseWithAngles[] PIXEL_STACK_POSES = {
            new PoseWithAngles(new Pose2d(-60, -23, Math.toRadians(180)), Math.toRadians(160), Math.toRadians(180)),
            new PoseWithAngles(new Pose2d(-60, 23, Math.toRadians(180)), Math.toRadians(-160), Math.toRadians(180))
    };

    /**
     * Use like PARK_POSES[alliance index]
     */
    public static final PoseWithAngles[] PARK_POSES = {
            new PoseWithAngles(new Pose2d(55, -60, Math.toRadians(180)), Math.toRadians(-90), Math.toRadians(180)),
            new PoseWithAngles(new Pose2d(60, 60, Math.toRadians(180)), Math.toRadians(90), Math.toRadians(180))
    };
}