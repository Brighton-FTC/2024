package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DroneLauncherComponent {
    Motor aeroplaneMotor;

    public DroneLauncherComponent(MotorEx paperAeroplaneMotor) {
        aeroplaneMotor = paperAeroplaneMotor;
    }

    public void launch() {
        aeroplaneMotor.setRunMode(Motor.RunMode.PositionControl);
        aeroplaneMotor.setTargetPosition(2000);

        while (!aeroplaneMotor.atTargetPosition()) {
            aeroplaneMotor.set(0.75);
        }
        aeroplaneMotor.stopMotor();
    }
}
