package org.firstinspires.ftc.teamcode.opMode.test.withDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Basic Autonomous - Red", group = "autonomous-test")
public class BasicAutonomousRed extends BasicAutonomousGeneric {
    @Override
    public void init() {
        teamColor = TeamColor.RED;
        super.init();
    }
}
