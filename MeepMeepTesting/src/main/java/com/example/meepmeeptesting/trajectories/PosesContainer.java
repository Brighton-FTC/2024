package com.example.meepmeeptesting.trajectories;


import static com.example.meepmeeptesting.trajectories.CyclePosesContainer.BLUE_CYCLE_POSES;
import static com.example.meepmeeptesting.trajectories.CyclePosesContainer.RED_CYCLE_POSES;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * A class to house trajectories and poses.
 */
public class PosesContainer {
    // TODO: fill in more precisely
    public static final PosesContainer RED_AUDIENCE_POSES = new PosesContainer(
            new Pose2d(-35, -60, Math.toRadians(90)), // startingPose
            new Pose2d(-35, -32, Math.toRadians(180)), // leftSpikeMarkPose
            new Pose2d(-35, -32, Math.toRadians(0)), // rightSpikeMarkPose
            new Pose2d(-35, -32, Math.toRadians(90)), // centerSpikeMarkPose
            // spikeToBackdropPart1Pose
            // spikeToBackdropPart1EndTangent
            // spikeToBackdropPart1SetTangent
            // spikeToBackdropPart2EndTangent
            // spikeToPixelPart1Pose
            // spikeToPixelPart1EndTangent
            // spikeToPixelPart1SetTangent
            // spikeToPixelPart2EndTangent
            // cyclePoses
            RED_CYCLE_POSES
    );

    public static final PosesContainer RED_FAR_SIDE_POSES = new PosesContainer(
            new Pose2d(10, -60, Math.toRadians(90)),
            new Pose2d(10, -30, Math.toRadians(180)),
            new Pose2d(10, -30, Math.toRadians(0)),
            new Pose2d(10, -30, Math.toRadians(90)),
            // startingPose
            // leftSpikeMarkPose
            // rightSpikeMarkPose
            // centerSpikeMarkPose
            // spikeToBackdropPart1Pose
            // spikeToBackdropPart1EndTangent
            // spikeToBackdropPart1SetTangent
            // spikeToBackdropPart2EndTangent
            // spikeToPixelPart1Pose
            // spikeToPixelPart1EndTangent
            // spikeToPixelPart1SetTangent
            // spikeToPixelPart2EndTangent
            // cyclePoses
            RED_CYCLE_POSES
    );
    public static final PosesContainer BLUE_AUDIENCE_POSES = new PosesContainer(
            new Pose2d(-35, 60, Math.toRadians(270)),

            new Pose2d(-35, 30, Math.toRadians(0)),
            new Pose2d(-35, 30, Math.toRadians(180)),
            new Pose2d(-35, 30, Math.toRadians(270)),
            // startingPose
            // leftSpikeMarkPose
            // rightSpikeMarkPose
            // centerSpikeMarkPose
            // spikeToBackdropPart1Pose
            // spikeToBackdropPart1EndTangent
            // spikeToBackdropPart1SetTangent
            // spikeToBackdropPart2EndTangent
            // spikeToPixelPart1Pose
            // spikeToPixelPart1EndTangent
            // spikeToPixelPart1SetTangent
            // spikeToPixelPart2EndTangent
            // cyclePoses
            BLUE_CYCLE_POSES
    );

    public static final PosesContainer BLUE_FAR_SIDE_POSES = new PosesContainer(
            new Pose2d(10, 60, Math.toRadians(270)),
            new Pose2d(10, 30, Math.toRadians(0)),
            new Pose2d(10, 30, Math.toRadians(180)),
            new Pose2d(10, 30, Math.toRadians(270)),
            // startingPose
            // leftSpikeMarkPose
            // rightSpikeMarkPose
            // centerSpikeMarkPose
            // spikeToBackdropPart1Pose
            // spikeToBackdropPart1EndTangent
            // spikeToBackdropPart1SetTangent
            // spikeToBackdropPart2EndTangent
            // spikeToPixelPart1Pose
            // spikeToPixelPart1EndTangent
            // spikeToPixelPart1SetTangent
            // spikeToPixelPart2EndTangent
            // cyclePoses
            BLUE_CYCLE_POSES
    );

    public static final PosesContainer[] POSES = {RED_AUDIENCE_POSES, RED_FAR_SIDE_POSES, BLUE_AUDIENCE_POSES, BLUE_FAR_SIDE_POSES};
    public static final PosesContainer[] RED_POSES = {RED_AUDIENCE_POSES, RED_FAR_SIDE_POSES};
    public static final PosesContainer[] BLUE_POSES = {BLUE_AUDIENCE_POSES, BLUE_FAR_SIDE_POSES};
    public static final PosesContainer[] AUDIENCE_POSES = {RED_AUDIENCE_POSES, BLUE_AUDIENCE_POSES};
    public static final PosesContainer[] FAR_SIDE_POSES = {RED_FAR_SIDE_POSES, BLUE_FAR_SIDE_POSES};

    public final Pose2d startingPose;
    public final Pose2d leftSpikeMarkPose;
    public final Pose2d rightSpikeMarkPose;
    public final Pose2d centerSpikeMarkPose;
    public final Pose2d spikeToBackdropPart1Pose;
    public final int spikeToBackdropPart1EndTangent;
    public final int spikeToBackdropPart1SetTangent;
    public final int spikeToBackdropPart2EndTangent;
    public final Pose2d spikeToPixelPart1Pose;
    public final int spikeToPixelPart1EndTangent;
    public final int spikeToPixelPart1SetTangent;
    public final int spikeToPixelPart2EndTangent;
    public final CyclePosesContainer cyclePoses;

    // arrays are in format [left, right, center]
    public final Pose2d[] backdropPoses;
    public final Pose2d[] spikeMarkPoses;


    public PosesContainer(Pose2d startingPose,
                          Pose2d leftSpikeMarkPose,
                          Pose2d rightSpikeMarkPose,
                          Pose2d centerSpikeMarkPose,
                          Pose2d spikeToBackdropPart1Pose,
                            int spikeToBackdropPart1EndTangent,
                            int spikeToBackdropPart1SetTangent,
                            int spikeToBackdropPart2EndTangent,
                            Pose2d spikeToPixelPart1Pose,
                            int spikeToPixelPart1EndTangent,
                            int spikeToPixelPart1SetTangent,
                            int spikeToPixelPart2EndTangent,
                            CyclePosesContainer cyclePoses) {
        this.startingPose = startingPose;
        this.leftSpikeMarkPose = leftSpikeMarkPose;
        this.rightSpikeMarkPose = rightSpikeMarkPose;
        this.centerSpikeMarkPose = centerSpikeMarkPose;
        this.cyclePoses = cyclePoses;
        this.spikeToBackdropPart1Pose = spikeToBackdropPart1Pose;
        this.spikeToBackdropPart1EndTangent = spikeToBackdropPart1EndTangent;
        this.spikeToBackdropPart1SetTangent = spikeToBackdropPart1SetTangent;
        this.spikeToBackdropPart2EndTangent = spikeToBackdropPart2EndTangent;
        this.spikeToPixelPart1Pose = spikeToPixelPart1Pose;
        this.spikeToPixelPart1EndTangent = spikeToPixelPart1EndTangent;
        this.spikeToPixelPart1SetTangent = spikeToPixelPart1SetTangent;
        this.spikeToPixelPart2EndTangent = spikeToPixelPart2EndTangent;

        this.backdropPoses = new Pose2d[]{cyclePoses.leftBackdropPose, cyclePoses.rightBackdropPose, cyclePoses.centerBackdropPose};
        this.spikeMarkPoses = new Pose2d[]{leftSpikeMarkPose, rightSpikeMarkPose, centerSpikeMarkPose};
    }
}