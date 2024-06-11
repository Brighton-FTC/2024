package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.meepmeeptesting.trajectories.Drive;
import com.example.meepmeeptesting.trajectories.PosesContainer;
import com.example.meepmeeptesting.trajectories.TrajectoriesFactory;
import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.RandomizationState;
import com.example.meepmeeptesting.util.StartingSide;
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

        for (AllianceColor alliance : AllianceColor.values()) {
            for (StartingSide startingSide : StartingSide.values()) {
                for (RandomizationState randomization : RandomizationState.values()) {
                    RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                            .setConstraints(constraints)
                            .setColorScheme(alliance == AllianceColor.BLUE ? new ColorSchemeBlueLight() : new ColorSchemeRedLight())
                            .build();

                    bot.runAction(generateTrajectorySequence(bot.getDrive(), alliance, startingSide, randomization, 1));
                    meepMeep.addEntity(bot);
                }


                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .start();
            }
        }
    }

    /**
     * Generate a trajectory sequence for the autonomous.
     *
     * @param drive                 The {@link DriveShim} object to apply the trajectory on.
     * @param alliance The alliance that the robot is on.
     * @param startingSide The side that the robot is starting on (also dictates whether the bot will go to the pixel stack or the backdrop first.
     * @param randomization The randomization state of the game.
     * @param repeatTimes How many times the robot will go from the backdrop to the pixel stack, and back.
     * @return An {@link Action} containing the trajectories.
     */
    private static Action generateTrajectorySequence(DriveShim drive, AllianceColor alliance, StartingSide startingSide, RandomizationState randomization, int repeatTimes) {
        TrajectoriesFactory factory = new TrajectoriesFactory(new DriveShimAdaptor(drive), alliance, startingSide, randomization);

        List<Action> actions = new ArrayList<>();

        actions.add(factory.startToSpike());

        if (startingSide == StartingSide.AUDIENCE_SIDE) {
            actions.add(factory.spikeToPixel());
            actions.add(factory.pixelToBackdrop());
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