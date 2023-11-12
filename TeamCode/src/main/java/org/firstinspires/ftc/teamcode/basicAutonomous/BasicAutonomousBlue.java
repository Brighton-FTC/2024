package org.firstinspires.ftc.teamcode.basicAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Basic Autonomous - Blue", group = "autonomous-test")
public class BasicAutonomousBlue extends BasicAutonomousGeneric {
    @Override
    public void init() {
        teamColor = TeamColor.BLUE;
        super.init();
    }
}
