package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.RandomizationState;
import com.example.meepmeeptesting.util.StartingSide;

import static com.example.meepmeeptesting.trajectories.PosesContainer.*;

/**
 * A class for generating trajectories.
 * TODO: fix trajectory overshoot.
 */
public class TrajectoriesFactory {
    private final Drive drive;

    private final AllianceColor alliance;
    private final StartingSide startingSide;
    private final RandomizationState randomization;

    /**
     * A class for generating trajectories.
     * @param drive The {@link Drive} object to generate the trajectories for.
     * @param alliance The {@link AllianceColor} of the robot.
     * @param startingSide The {@link StartingSide} of the robot.
     * @param randomization The {@link RandomizationState} of the match.
     */
    public TrajectoriesFactory(Drive drive, AllianceColor alliance, StartingSide startingSide, RandomizationState randomization) {
        this.drive = drive;
        this.alliance = alliance;
        this.startingSide = startingSide;
        this.randomization = randomization;
    }

    public Action startToSpike() {
        Pose2d startingPose = STARTING_POSES[alliance.ordinal()][startingSide.ordinal()];
        PoseWithAngles spikeMarkPose = SPIKE_MARK_POSES[alliance.ordinal()][startingSide.ordinal()][randomization.ordinal()];

        return drive.actionBuilder(startingPose)
                .setTangent(spikeMarkPose.getTangent())
                .splineToLinearHeading(spikeMarkPose.getPose2d(), spikeMarkPose.getHeading())
                .build();
    }

    public Action spikeToBackdrop() {
        PoseWithAngles spikeMarkPose = SPIKE_MARK_POSES[alliance.ordinal()][startingSide.ordinal()][randomization.ordinal()];
        PoseWithAngles backdropPose = BACKDROP_POSES[alliance.ordinal()][randomization.ordinal()];

        return drive.actionBuilder(spikeMarkPose.getPose2d())
//                .splineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(180)), Math.toRadians(45))
                .setTangent(backdropPose.getTangent())
                .splineToLinearHeading(backdropPose.getPose2d(), backdropPose.getHeading())
                .build();
    }

    public Action spikeToPixel() {
        PoseWithAngles spikeMarkPose = SPIKE_MARK_POSES[alliance.ordinal()][startingSide.ordinal()][randomization.ordinal()];
        PoseWithAngles pixelStackPose = PIXEL_STACK_POSES[alliance.ordinal()];

        return drive.actionBuilder(spikeMarkPose.getPose2d())
                .setTangent(pixelStackPose.getTangent())
                .splineToLinearHeading(pixelStackPose.getPose2d(), pixelStackPose.getHeading())
                .build();
    }

    public Action backdropToPixel() {
        PoseWithAngles backdropPose = BACKDROP_POSES[alliance.ordinal()][randomization.ordinal()];
        PoseWithAngles pixelStackPose = PIXEL_STACK_POSES[alliance.ordinal()];

        return drive.actionBuilder(backdropPose.getPose2d())
//                .splineToSplineHeading(new Pose2d(16, -9, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(pixelStackPose.getTangent())
                .splineToLinearHeading(pixelStackPose.getPose2d(), pixelStackPose.getHeading())
                .build();
    }

    public Action pixelToBackdrop() {
        PoseWithAngles pixelStackPose = PIXEL_STACK_POSES[alliance.ordinal()];
        PoseWithAngles backdropPose = BACKDROP_POSES[alliance.ordinal()][randomization.ordinal()];

        return drive.actionBuilder(pixelStackPose.getPose2d())
//                .setTangent(Math.toRadians(30))
//                .splineToSplineHeading(new Pose2d(-16, -9, Math.toRadians(180)), Math.toRadians(360))
                .setTangent(backdropPose.getTangent())
                .splineToLinearHeading(backdropPose.getPose2d(), backdropPose.getHeading())
                .build();
    }

    public Action park() {
        PoseWithAngles backdropPose = BACKDROP_POSES[alliance.ordinal()][randomization.ordinal()];
        PoseWithAngles parkPose = PARK_POSES[alliance.ordinal()];

        return drive.actionBuilder(backdropPose.getPose2d())
                .setTangent(parkPose.getTangent())
                .splineToLinearHeading(parkPose.getPose2d(), parkPose.getHeading())
                .build();
    }
}
