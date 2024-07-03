package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Red Audience Side", group = "autonomous")
public class AutonomousRedAudienceSide extends AutonomousGeneric {
    @Override
    protected void setColorSide() {
        this.alliance = AllianceColor.RED;
        this.startingSide = StartingSide.AUDIENCE_SIDE;
    }
}
