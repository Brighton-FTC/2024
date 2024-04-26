package org.firstinspires.ftc.teamcode.opModes.autonomous;


import com.acmerobotics.roadrunner.Pose2d;

/**
 * A class to house trajectories and poses.
 */
public class PosesContainer {
    // TODO: fill in
    public static final PosesContainer RED_AUDIENCE_POSES = null;
    public static final PosesContainer RED_FAR_SIDE_POSES = null;
    public static final PosesContainer BLUE_AUDIENCE_POSES = null;
    public static final PosesContainer BLUE_FAR_SIDE_POSES = null;

    public final Pose2d startingPose;
    public final Pose2d leftSpikeMarkPose;
    public final Pose2d rightSpikeMarkPose;
    public final Pose2d centerSpikeMarkPose;
    public final Pose2d leftBackdropPose;
    public final Pose2d rightBackdropPose;
    public final Pose2d centerBackdropPose;
    public final Pose2d pixelStackPose;
    public final Pose2d parkPose;

    public PosesContainer(Pose2d startingPose,
                          Pose2d leftSpikeMarkPose,
                          Pose2d rightSpikeMarkPose,
                          Pose2d centerSpikeMarkPose,
                          Pose2d leftBackdropPose,
                          Pose2d rightBackdropPose,
                          Pose2d centerBackdropPose,
                          Pose2d pixelStackPose,
                          Pose2d parkPose) {
        this.startingPose = startingPose;
        this.leftSpikeMarkPose = leftSpikeMarkPose;
        this.rightSpikeMarkPose = rightSpikeMarkPose;
        this.centerSpikeMarkPose = centerSpikeMarkPose;
        this.leftBackdropPose = leftBackdropPose;
        this.rightBackdropPose = rightBackdropPose;
        this.centerBackdropPose = centerBackdropPose;
        this.pixelStackPose = pixelStackPose;
        this.parkPose = parkPose;
    }
}