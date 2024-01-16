package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DroneLauncherCompnenet {
    private GamepadEx driverOp = new GamepadEx(gamepad1);
    Motor aeroplaneMotor;
    public DroneLauncherCompnenet(MotorEx paperAeroplaneMotor) {
        aeroplaneMotor = paperAeroplaneMotor;

    }
    public void launch() {
        aeroplaneMotor.setRunMode(Motor.RunMode.PositionControl);
        aeroplaneMotor.setPositionCoefficient(0.05);
        double kP = aeroplaneMotor.getPositionCoefficient();
        aeroplaneMotor.setTargetPosition(2000);

        aeroplaneMotor.set(0);

        aeroplaneMotor.setPositionTolerance(13.6);
        while (!aeroplaneMotor.atTargetPosition()) {
            aeroplaneMotor.set(0.75);
        }
        aeroplaneMotor.stopMotor();
        aeroplaneMotor.setDistancePerPulse(0.015);

        aeroplaneMotor.setTargetDistance(18.0);

        aeroplaneMotor.set(0.5);
    }


}
