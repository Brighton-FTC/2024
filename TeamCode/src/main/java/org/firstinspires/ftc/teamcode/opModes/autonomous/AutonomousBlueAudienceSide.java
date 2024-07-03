package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Blue Audience Side", group = "autonomous")
public class AutonomousBlueAudienceSide extends AutonomousGeneric{
    @Override
    protected void setColorSide() {
        this.alliance = AllianceColor.BLUE;
        this.startingSide = StartingSide.AUDIENCE_SIDE;
    }
}
