package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

/**
 * A class for generating trajectories.
 * TODO: fix trajectory overshoot.
 */
public class TrajectoriesFactory {
    private final Drive drive;
    private final PosesContainer poses;
    private final Pose2d startPose, spikeMarkPose, backdropThirdPose;


    /**
     * A class for generating trajectories.
     * @param drive The {@link Drive} object to generate the trajectories for.
     * @param poses The {@link PosesContainer} object that contains most of the poses.
     * @param startPose The starting pose of the robot (if unspecified, will default to {@link PosesContainer#startingPose}.
     * @param spikeMarkPose The spike mark that the robot should drive to.
     * @param backdropThirdPose The third of the backdrop that the robot should drive to.
     */
    public TrajectoriesFactory(Drive drive, PosesContainer poses, Pose2d startPose, Pose2d spikeMarkPose, Pose2d backdropThirdPose) {
        this.drive = drive;
        this.poses = poses;
        this.startPose = startPose;
        this.spikeMarkPose = spikeMarkPose;
        this.backdropThirdPose = backdropThirdPose;
    }

    /**
     * A class for generating trajectories.
     * @param drive The {@link Drive} object to generate the trajectories for.
     * @param poses The {@link PosesContainer} object that contains most of the poses.
     * @param spikeMarkPose The spike mark that the robot should drive to.
     * @param backdropThirdPose The third of the backdrop that the robot should drive to.
     */
    public TrajectoriesFactory(Drive drive, PosesContainer poses, Pose2d spikeMarkPose, Pose2d backdropThirdPose) {
        this(drive, poses, poses.startingPose, spikeMarkPose, backdropThirdPose);
    }

    public Action driveToSpikeMark() {
        return drive.actionBuilder(startPose)
                .splineToLinearHeading(spikeMarkPose, spikeMarkPose.heading)
                .build();
    }

    public Action driveToBackdropFromSpikeMarks() {
        return drive.actionBuilder(spikeMarkPose)
                .splineToLinearHeading(backdropThirdPose, backdropThirdPose.heading)
                .build();
    }

    public Action driveToPixelStackFromSpikeMarks() {
        return drive.actionBuilder(spikeMarkPose)
                .splineToLinearHeading(poses.pixelStackPose, poses.pixelStackPose.heading)
                .build();
    }

    public Action driveToPixelStackFromBackdrop() {
        return drive.actionBuilder(backdropThirdPose)
                .splineToLinearHeading(poses.pixelStackPose, poses.pixelStackPose.heading)
                .build();
    }

    public Action driveToBackdropFromPixelStack() {
        return drive.actionBuilder(poses.pixelStackPose)
                .splineToLinearHeading(backdropThirdPose, backdropThirdPose.heading)
                .build();
    }

    public Action parkFromBackdrop() {
        return drive.actionBuilder(backdropThirdPose)
                .splineToLinearHeading(poses.parkPose, poses.parkPose.heading)
                .build();
    }
}
