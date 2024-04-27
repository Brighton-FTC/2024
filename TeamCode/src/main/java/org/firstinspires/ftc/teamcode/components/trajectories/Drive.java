package org.firstinspires.ftc.teamcode.components.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

/**
 * An interface for drives. <br />
 * Used for uniting MecanumDrive and DriveShim classes.
 */
public interface Drive {
    TrajectoryActionBuilder actionBuilder(Pose2d beginPose);
}
