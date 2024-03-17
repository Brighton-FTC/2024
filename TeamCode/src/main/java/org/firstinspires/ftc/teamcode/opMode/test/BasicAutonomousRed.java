package org.firstinspires.ftc.teamcode.opMode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Basic Autonomous - Red", group = "autonomous-test")
public class BasicAutonomousRed extends BasicAutonomousGeneric {
    @Override
    public void init() {
        teamColor = TeamColor.RED;
        backdropTurningAngle = -90;

        aprilTagIds = new int[]{4, 5, 6};

        cvLower = new Scalar(355, 120, 205);
        cvUpper = new Scalar(20, 245, 255);

        super.init();
    }
}
