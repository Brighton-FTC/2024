package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * A class to house trajectories and poses.
 */
public class PosesContainer {
    private final Pose2d startingPose;
    private final Pose2d leftSpikeMarkPose;
    private final Pose2d rightSpikeMarkPose;
    private final Pose2d centerSpikeMarkPose;
    private final Pose2d leftBackdropPose;
    private final Pose2d rightBackdropPose;
    private final Pose2d centerBackdropPose;
    private final Pose2d pixelStackPose;
    private final Pose2d parkPose;

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

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public Pose2d getLeftSpikeMarkPose() {
        return leftSpikeMarkPose;
    }

    public Pose2d getRightSpikeMarkPose() {
        return rightSpikeMarkPose;
    }

    public Pose2d getCenterSpikeMarkPose() {
        return centerSpikeMarkPose;
    }

    public Pose2d getLeftBackdropPose() {
        return leftBackdropPose;
    }

    public Pose2d getRightBackdropPose() {
        return rightBackdropPose;
    }

    public Pose2d getCenterBackdropPose() {
        return centerBackdropPose;
    }

    public Pose2d getPixelStackPose() {
        return pixelStackPose;
    }

    public Pose2d getParkPose() {
        return parkPose;
    }
}
