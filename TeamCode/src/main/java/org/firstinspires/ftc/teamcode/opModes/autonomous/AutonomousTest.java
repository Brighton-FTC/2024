package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.meepmeeptesting.trajectories.Drive;
import com.example.meepmeeptesting.trajectories.TrajectoriesFactory;
import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.RandomizationState;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.roadRunner.MecanumDrive;

import static com.example.meepmeeptesting.trajectories.PosesContainer.*;

public class AutonomousTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSES[AllianceColor.RED.ordinal()][StartingSide.AUDIENCE_SIDE.ordinal()]);
        TrajectoriesFactory factory = new TrajectoriesFactory(new MecanumDriveAdaptor(drive), AllianceColor.RED, StartingSide.AUDIENCE_SIDE, RandomizationState.CENTER);

        waitForStart();
        Actions.runBlocking(factory.startToSpike());
        Actions.runBlocking(factory.audienceSpikeToBackdrop());
        Actions.runBlocking(factory.park());
    }

    private static class MecanumDriveAdaptor implements Drive {
        private final MecanumDrive drive;

        public MecanumDriveAdaptor(MecanumDrive drive) {
            this.drive = drive;
        }

        @Override
        public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
            return drive.actionBuilder(beginPose);
        }
    }
}
