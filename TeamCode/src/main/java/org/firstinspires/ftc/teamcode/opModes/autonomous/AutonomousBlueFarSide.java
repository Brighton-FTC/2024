package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.example.meepmeeptesting.util.AllianceColor;
import com.example.meepmeeptesting.util.StartingSide;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Blue Far Side", group = "autonomous")
public class AutonomousBlueFarSide extends AutonomousGeneric {
    protected AutonomousBlueFarSide() {
        super(AllianceColor.BLUE, StartingSide.FAR_SIDE);
    }
}
