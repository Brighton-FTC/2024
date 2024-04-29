package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

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
                .setTangent(backdropThirdPose.heading)
                .strafeTo(backdropThirdPose.position)
                .build();
    }

    public Action driveToPixelStackFromSpikeMarks() {
        return drive.actionBuilder(spikeMarkPose)
                .setTangent(poses.pixelStackPose.heading)
                .strafeTo(poses.pixelStackPose.position)
                .build();
    }

    public Action driveToPixelStackFromBackdrop() {
        return drive.actionBuilder(backdropThirdPose)
                .setTangent(poses.pixelStackPose.heading)
                .strafeTo(poses.pixelStackPose.position)
                .build();
    }

    public Action driveToBackdropFromPixelStack() {
        return drive.actionBuilder(poses.pixelStackPose)
                .setTangent(backdropThirdPose.heading)
                .strafeTo(backdropThirdPose.position)
                .build();
    }

    public Action parkFromBackdrop() {
        return drive.actionBuilder(backdropThirdPose)
                .strafeTo(new Vector2d(backdropThirdPose.position.x, poses.parkPose.position.y))
                .strafeTo(poses.parkPose.position)
                .build();
    }
}
