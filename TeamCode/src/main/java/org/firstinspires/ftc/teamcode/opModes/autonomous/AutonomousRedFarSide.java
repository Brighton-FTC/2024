package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Red Far Side", group = "autonomous")
public class AutonomousRedFarSide extends AutonomousGeneric {

    @Override
    protected void setColorSide() {
        this.alliance = AllianceColor.RED;
        this.startingSide = StartingSide.FAR_SIDE;
    }
}
