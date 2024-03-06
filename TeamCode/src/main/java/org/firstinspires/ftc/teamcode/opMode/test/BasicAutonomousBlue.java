package org.firstinspires.ftc.teamcode.opMode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Scalar;

@Disabled
@Autonomous(name = "Basic Autonomous - Blue", group = "autonomous-test")
public class BasicAutonomousBlue extends BasicAutonomousGeneric {
    @Override
    public void init() {
        teamColor = TeamColor.BLUE;
        backdropTurningAngle = -90;

        cvLower = new Scalar(80, 10, 150);
        cvUpper = new Scalar(110, 100, 255);

        super.init();
    }
}
