package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Red Audience Side", group = "autonomous")
public class AutonomousRedAudienceSide extends AutonomousGeneric {
    protected AutonomousRedAudienceSide() {
        super(AllianceColor.RED, StartingSide.AUDIENCE_SIDE);
    }
}
