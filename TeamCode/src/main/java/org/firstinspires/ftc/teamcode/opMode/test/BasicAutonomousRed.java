package org.firstinspires.ftc.teamcode.opMode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Scalar;

@Disabled
@Autonomous(name = "Basic Autonomous - Red", group = "autonomous-test")
public class BasicAutonomousRed extends BasicAutonomousGeneric {
    @Override
    public void init() {
        teamColor = TeamColor.RED;
        backdropTurningAngle = 90;
        cvLower = new Scalar(350, 160, 90);
        cvUpper = new Scalar(20, 245, 255);

        super.init();
    }
}
