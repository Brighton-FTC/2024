package org.firstinspires.ftc.teamcode.opMode.test.withoutDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Basic Autonomous - Blue", group = "autonomous-test")
public class BasicAutonomousBlue extends BasicAutonomousGeneric {
    @Override
    public void init() {
        WANTED_LABELS = new String[]{"Pixel", "blue_cone"};
        teamColor = TeamColor.BLUE;
        super.init();
    }
}
