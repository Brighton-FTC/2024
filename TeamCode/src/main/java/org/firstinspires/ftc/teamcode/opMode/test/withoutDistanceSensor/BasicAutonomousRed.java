package org.firstinspires.ftc.teamcode.opMode.test.withoutDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Basic Autonomous - Red", group = "autonomous-test")
public class BasicAutonomousRed extends BasicAutonomousGeneric {
    @Override
    public void init() {
        WANTED_LABELS = new String[]{"Pixel", "red_cone"};
        teamColor = TeamColor.RED;
        super.init();
    }
}
