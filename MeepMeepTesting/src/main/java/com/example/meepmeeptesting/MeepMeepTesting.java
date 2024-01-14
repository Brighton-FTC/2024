package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.TrajectoryPositions.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);


        for (int i = 0; i < 12; i++) {
            int finalI = i;
            RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                    .setDimensions(15, 15)
//                    .setConstraints(new Constraints()) // TODO: set constraints
                    .followTrajectorySequence(drive -> buildTrajectory(drive, finalI, 1));
            meepMeep.addEntity(bot);
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .start();
    }

    /**
     * Builds robot trajectory. Example use: <br />
     * <code>
     * defaultBotBuilder.followTrajectory(drive -> buildTrajectory(drive, 0, 3));
     * </code>
     *
     * @param drive             The {@link DriveShim} object.
     * @param robotId           Which path the robot follows, from 0 to 11 (inclusive).
     * @param pixelCollectTimes Make the robot collect a pixel this many times (for far starting side),
     *                          or this many times plus one (for audience starting side).
     * @return A {@link TrajectorySequence} object, containing the robot's trajectory.
     */
    @NotNull
    private static TrajectorySequence buildTrajectory(@NotNull DriveShim drive, int robotId, int pixelCollectTimes) {
        if (robotId < 0 || robotId > 11) {
            throw new IllegalArgumentException("Parameter 'robotId' must be between 0 and 11 (inclusive).");
        }

        Pose2d startPose;
        if (robotId < 3) {
            startPose = STARTING_POSE_RED_AUDIENCE;
        } else if (robotId < 6) {
            startPose = STARTING_POSE_RED_FAR;
        } else if (robotId < 9) {
            startPose = STARTING_POSE_BLUE_AUDIENCE;
        } else {
            startPose = STARTING_POSE_BLUE_FAR;
        }
        TrajectorySequenceBuilder trajectory = drive.trajectorySequenceBuilder(startPose);

        Pose2d spikeMarkPose = new Pose2d();
        Pose2d backdropPose = new Pose2d();

        switch (robotId) {
            case 0:
                spikeMarkPose = SPIKE_MARK_RED_AUDIENCE_LEFT;
                backdropPose = BACKDROP_RED_LEFT;
                break;

            case 1:
                spikeMarkPose = SPIKE_MARK_RED_AUDIENCE_CENTER;
                backdropPose = BACKDROP_RED_CENTER;
                break;

            case 2:
                spikeMarkPose = SPIKE_MARK_RED_AUDIENCE_RIGHT;
                backdropPose = BACKDROP_RED_RIGHT;
                break;

            case 3:
                spikeMarkPose = SPIKE_MARK_RED_FAR_LEFT;
                backdropPose = BACKDROP_RED_LEFT;
                break;

            case 4:
                spikeMarkPose = SPIKE_MARK_RED_FAR_CENTER;
                backdropPose = BACKDROP_RED_CENTER;
                break;

            case 5:
                spikeMarkPose = SPIKE_MARK_RED_FAR_RIGHT;
                backdropPose = BACKDROP_RED_RIGHT;
                break;

            case 6:
                spikeMarkPose = SPIKE_MARK_BLUE_AUDIENCE_LEFT;
                backdropPose = BACKDROP_BLUE_LEFT;
                break;

            case 7:
                spikeMarkPose = SPIKE_MARK_BLUE_AUDIENCE_CENTER;
                backdropPose = BACKDROP_BLUE_CENTER;
                break;

            case 8:
                spikeMarkPose = SPIKE_MARK_BLUE_AUDIENCE_RIGHT;
                backdropPose = BACKDROP_BLUE_RIGHT;
                break;

            case 9:
                spikeMarkPose = SPIKE_MARK_BLUE_FAR_LEFT;
                backdropPose = BACKDROP_BLUE_LEFT;
                break;

            case 10:
                spikeMarkPose = SPIKE_MARK_BLUE_FAR_CENTER;
                backdropPose = BACKDROP_BLUE_CENTER;
                break;

            case 11:
                spikeMarkPose = SPIKE_MARK_BLUE_FAR_RIGHT;
                backdropPose = BACKDROP_BLUE_RIGHT;
        }

        trajectory.splineTo(spikeMarkPose.vec(), spikeMarkPose.getHeading());

        Pose2d pixelStackPose = robotId >= 6 ? PIXEL_STACK_LEFT_1 : PIXEL_STACK_RIGHT_1;


        if ((robotId < 3) || (robotId >= 6 && robotId < 9)) {
            trajectory.lineToLinearHeading(pixelStackPose);
            trajectory.addDisplacementMarker(() -> {/* pick up pixel */});
        }

        trajectory.lineToLinearHeading(backdropPose);

        for (int i = 0; i < pixelCollectTimes; i++) {
            trajectory.addDisplacementMarker(() -> {/* place pixel */});
            trajectory.lineToLinearHeading(pixelStackPose);
            trajectory.addDisplacementMarker(() -> {/* pick up pixel */});
            trajectory.lineToLinearHeading(backdropPose);
        }

        return trajectory.build();
    }
}