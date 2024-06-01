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
    private final Pose2d startPose, spikeMarkPose, backdropPose;


    /**
     * A class for generating trajectories.
     * @param drive The {@link Drive} object to generate the trajectories for.
     * @param poses The {@link PosesContainer} object that contains most of the poses.
     * @param startPose The starting pose of the robot (if unspecified, will default to {@link PosesContainer#startingPose}.
     * @param spikeMarkPose The spike mark that the robot should drive to.
     * @param backdropPose The third of the backdrop that the robot should drive to.
     */
    public TrajectoriesFactory(Drive drive, PosesContainer poses, Pose2d startPose, Pose2d spikeMarkPose, Pose2d backdropPose) {
        this.drive = drive;
        this.poses = poses;
        this.startPose = startPose;
        this.spikeMarkPose = spikeMarkPose;
        this.backdropPose = backdropPose;
    }

    /**
     * A class for generating trajectories.
     * @param drive The {@link Drive} object to generate the trajectories for.
     * @param poses The {@link PosesContainer} object that contains most of the poses.
     * @param spikeMarkPose The spike mark that the robot should drive to.
     * @param backdropPose The third of the backdrop that the robot should drive to.
     */
    public TrajectoriesFactory(Drive drive, PosesContainer poses, Pose2d spikeMarkPose, Pose2d backdropPose) {
        this(drive, poses, poses.startingPose, spikeMarkPose, backdropPose);
    }

    public Action startToSpike() {
        return drive.actionBuilder(startPose)
                .splineToLinearHeading(spikeMarkPose, spikeMarkPose.heading)
                .build();
    }

    public Action spikeToBackdrop() {
        return drive.actionBuilder(spikeMarkPose)
                .splineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(180)), Math.toRadians(45))
                .setTangent(0)
                .splineToLinearHeading(backdropPose, Math.toRadians(0))
                .build();
    }

    public Action spikeToPixel() {
        return drive.actionBuilder(spikeMarkPose)
                .setTangent(poses.pixelStackPose.heading)
                .strafeTo(poses.pixelStackPose.position)
                .build();
    }

    public Action backdropToPixel() {
        return drive.actionBuilder(backdropPose)
                .splineToSplineHeading(new Pose2d(16, -9, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(poses.pixelStackPose, Math.toRadians(225))
                .build();
    }

    public Action pixelToBackdrop() {
        return drive.actionBuilder(poses.pixelStackPose)
                .setTangent(Math.toRadians(30))
                .splineToSplineHeading(new Pose2d(-16, -9, Math.toRadians(180)), Math.toRadians(360))
                .setTangent(Math.toRadians(360))
                .splineToLinearHeading(backdropPose, Math.toRadians(315))
                .build();
    }

    public Action park() {
        return drive.actionBuilder(backdropPose)
                .splineToLinearHeading(poses.parkPose, Math.toRadians(0))
                .build();
    }
}
