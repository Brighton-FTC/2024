package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * Code to launch the paper drone.
 */
public class DroneLauncherComponent {
    Motor aeroplaneMotor;

    // TODO: Tune this
    public static final int TARGET_POSITION = 2000;

    /**
     * Code to launch the paper drone.
     *
     * @param paperAeroplaneMotor Motor for drone launcher.
     */
    public DroneLauncherComponent(MotorEx paperAeroplaneMotor) {
        aeroplaneMotor = paperAeroplaneMotor;
    }

    /**
     * Sets the motor's setpoint and moves motor until at setpoint.
     * Only needs to be called once, but can be called multiple times without anything happening.
     */
    public void launch() {
        aeroplaneMotor.setRunMode(Motor.RunMode.PositionControl);
        aeroplaneMotor.setTargetPosition(TARGET_POSITION);

        while (!aeroplaneMotor.atTargetPosition()) {
            aeroplaneMotor.set(0.75);
        }
        aeroplaneMotor.stopMotor();
    }
}

