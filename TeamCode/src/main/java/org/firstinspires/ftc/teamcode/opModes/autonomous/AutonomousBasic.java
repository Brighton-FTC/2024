package org.firstinspires.ftc.teamcode.opModes.autonomous;

import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.MIN_DETECTION_AREA;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.blueLower;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.blueUpper;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.redLower;
import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.redUpper;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.example.meepmeeptesting.trajectories.PosesContainer;
import com.example.meepmeeptesting.util.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.vision.eocv.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.util.roadRunner.MecanumDrive;
import org.opencv.core.Scalar;

public class AutonomousBasic extends LinearOpMode {
    protected AllianceColor alliance;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ActiveIntakeComponent activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "active_intake_motor_left"));


        Scalar LOWER_DETECTION_BOUND = alliance == AllianceColor.BLUE ? blueLower : redLower;
        Scalar UPPER_DETECTION_BOUND = alliance == AllianceColor.BLUE ? blueUpper : redUpper;

        ColourMassDetectionProcessor colorMassDetectionProcessor = new ColourMassDetectionProcessor(
                LOWER_DETECTION_BOUND,
                UPPER_DETECTION_BOUND,
                () -> MIN_DETECTION_AREA,
                () -> CAMERA_RESOLUTION.getWidth() / 3.0,
                () -> CAMERA_RESOLUTION.getWidth() * 2.0 / 3.0
        );

        waitForStart();

        ColourMassDetectionProcessor.PropPositions propPosition = colorMassDetectionProcessor.getRecordedPropPosition();

        Pose2d spikeMarkPose = PosesContainer.SPIKE_MARK_POSES[0][0][propPosition.id].getPose2d();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(PosesContainer.STARTING_POSES[0][0])
                        .splineTo(spikeMarkPose.position, spikeMarkPose.heading)
                        .build(),
                telemetryPacket -> {
                    activeIntake.turnBackwards();
                    return true;
                }
        ));
    }
}
