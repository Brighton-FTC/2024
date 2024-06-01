package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.meepmeeptesting.trajectories.Drive;
import com.example.meepmeeptesting.trajectories.PosesContainer;
import com.example.meepmeeptesting.trajectories.TrajectoriesFactory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(600);
        Constraints constraints = new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15);

        for (int randomization = 0; randomization < 3; randomization++) {
            RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(constraints)
                    .setColorScheme(Arrays.asList(PosesContainer.RED_POSES).contains(PosesContainer.RED_AUDIENCE_POSES) ? new ColorSchemeRedLight() : new ColorSchemeBlueLight())
                    .build();

            bot.runAction(generateTrajectorySequence(bot.getDrive(), PosesContainer.RED_AUDIENCE_POSES, randomization, 3, Arrays.asList(PosesContainer.AUDIENCE_POSES).contains(constraints)));
            meepMeep.addEntity(bot);
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .start();
    }

    /**
     * Generate a trajectory sequence for the autonomous.
     *
     * @param drive                 The {@link DriveShim} object to apply the trajectory on.
     * @param poses                 The {@link Pose2d} object to draw the poses from.
     * @param randomization         An integer from 0 to 2 (inclusive), which represents the randomization (left, right, or center).
     * @param repeatTimes           The number of times to go to the pixel stacks and then to the backdrop after the randomization tasks have been completed.
     * @param goesToPixelStackFirst Whether the robot goes to the pixel stack, and then finishes the randomization tasks or not.
     * @return An {@link Action} containing the trajectories.
     */
    private static Action generateTrajectorySequence(DriveShim drive, PosesContainer poses, int randomization, int repeatTimes, boolean goesToPixelStackFirst) {
        Pose2d spikeMarkPose, backdropThirdPose;

        switch (randomization) {
            case 0:
                spikeMarkPose = poses.leftSpikeMarkPose;
                backdropThirdPose = poses.leftBackdropPose;
                break;
            case 1:
                spikeMarkPose = poses.rightSpikeMarkPose;
                backdropThirdPose = poses.rightBackdropPose;
                break;
            case 2:
                spikeMarkPose = poses.centerSpikeMarkPose;
                backdropThirdPose = poses.centerBackdropPose;
                break;
            default:
                throw new IllegalArgumentException("'randomization' must be between 0 and 2 (inclusive), but was " + randomization);
        }

        TrajectoriesFactory factory = new TrajectoriesFactory(new DriveShimAdaptor(drive), poses, spikeMarkPose, backdropThirdPose);

        List<Action> actions = new ArrayList<>();

        actions.add(factory.startToSpike());

        if (goesToPixelStackFirst) {
            actions.add(factory.spikeToPixel());
            actions.add(factory.pixelToBackdrop());
//            actions.add(factory.driveToBackdropFromPixelStack());
        } else {
            actions.add(factory.spikeToBackdrop());
        }

        for (int i = 0; i < repeatTimes; i++) {
            actions.add(factory.backdropToPixel());
            actions.add(new SleepAction(1.0));
            actions.add(factory.pixelToBackdrop());
            actions.add(new SleepAction(1.0));
        }

        actions.add(factory.park());

        return new SequentialAction(actions);
    }

    private static class DriveShimAdaptor implements Drive {
        private final DriveShim drive;

        public DriveShimAdaptor(DriveShim drive) {
            this.drive = drive;
        }

        @Override
        public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
            return drive.actionBuilder(beginPose);
        }
    }
}