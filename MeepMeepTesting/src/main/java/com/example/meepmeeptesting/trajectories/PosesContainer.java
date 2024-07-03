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
                            new PoseWithAngles(new Pose2d(-35 + 2, -30, Math.toRadians(0)), Math.toRadians(90)),
                            new PoseWithAngles(new Pose2d(-35 - 2, -30, Math.toRadians(180)), Math.toRadians(90)),
                            new PoseWithAngles(new Pose2d(-35, -32 - 6, Math.toRadians(270)), Math.toRadians(90))
                    },
                    {

                            new PoseWithAngles(new Pose2d(10 + 2, -30, Math.toRadians(0)), Math.toRadians(90)),
                            new PoseWithAngles(new Pose2d(10 - 2, -30, Math.toRadians(180)), Math.toRadians(90)),
                            new PoseWithAngles(new Pose2d(10, -30 - 6, Math.toRadians(270)), Math.toRadians(90)),
                    }
            },
            {
                    {

                            new PoseWithAngles(new Pose2d(-35 - 2, 30, Math.toRadians(180)), Math.toRadians(270)),
                            new PoseWithAngles(new Pose2d(-35 + 2, 30, Math.toRadians(0)), Math.toRadians(270)),
                            new PoseWithAngles(new Pose2d(-35, 30 + 6, Math.toRadians(90)), Math.toRadians(270))
                    },
                    {
                            new PoseWithAngles(new Pose2d(10 - 2, 30, Math.toRadians(180)), Math.toRadians(270)),
                            new PoseWithAngles(new Pose2d(10 + 2, 30, Math.toRadians(0)), Math.toRadians(270)),
                            new PoseWithAngles(new Pose2d(10, 30 + 6, Math.toRadians(90)), Math.toRadians(270))
                    }
            }
    };

    /**
     * Use like BACKDROP_POSES[alliance index][randomization index]
     */
    public static final PoseWithAngles[][] AUDIENCE_BACKDROP_POSES = {
            {
                    new PoseWithAngles(new Pose2d(47, -30, Math.toRadians(180)), Math.toRadians(270), Math.toRadians(90)),
                    new PoseWithAngles(new Pose2d(47, -40, Math.toRadians(180)), Math.toRadians(270), Math.toRadians(90)),
                    new PoseWithAngles(new Pose2d(47, -35, Math.toRadians(180)), Math.toRadians(270), Math.toRadians(90))
            },
            {
                    new PoseWithAngles(new Pose2d(47, 40, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90)),
                    new PoseWithAngles(new Pose2d(47, 30, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90)),
                    new PoseWithAngles(new Pose2d(47, 35, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90))
            }
    };

    /**
     * Use like BACKDROP_POSES[alliance index][randomization index]
     */
    public static final PoseWithAngles[][] FAR_BACKDROP_POSES = {
            {
                    new PoseWithAngles(new Pose2d(47, -30, Math.toRadians(180)), Math.toRadians(90), Math.toRadians(270)),
                    new PoseWithAngles(new Pose2d(47, -40, Math.toRadians(180)), Math.toRadians(90), Math.toRadians(270)),
                    new PoseWithAngles(new Pose2d(47, -35, Math.toRadians(180)), Math.toRadians(90), Math.toRadians(270))
            },
            {
                    new PoseWithAngles(new Pose2d(47, 40, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90)),
                    new PoseWithAngles(new Pose2d(47, 30, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90)),
                    new PoseWithAngles(new Pose2d(47, 35, Math.toRadians(180)), Math.toRadians(-270), Math.toRadians(-90))
            }
    };

    /**
     * Use like PIXEL_STACK_POSES[alliance index]
     */
    public static final PoseWithAngles[] PIXEL_STACK_POSES = {
            new PoseWithAngles(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(90), Math.toRadians(270)),
            new PoseWithAngles(new Pose2d(-60, 36, Math.toRadians(180)), Math.toRadians(-90), Math.toRadians(270))
    };

    public static final PoseWithAngles[] AUDIENCE_LEFT_POSES = {
            new PoseWithAngles(new Pose2d(-34, -16, Math.toRadians(0)), Math.toRadians(90), Math.toRadians(270)),
            new PoseWithAngles(new Pose2d(-34, 16, Math.toRadians(180)), Math.toRadians(-90), Math.toRadians(270))
    };

    /**
     * Use like PARK_POSES[alliance index]
     */
    public static final PoseWithAngles[] PARK_POSES = {
            new PoseWithAngles(new Pose2d(55, -60, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(270)),
            new PoseWithAngles(new Pose2d(55, 60, Math.toRadians(180)), Math.toRadians(0), Math.toRadians(270))
    };
}