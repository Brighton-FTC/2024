package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Code to launch the paper drone.
 */
public class DroneLauncherComponent {
    ServoEx droneServo;

    // TODO: Tune this
    // Should turn 120 to 180 degrees (according to engineers)
    public static final int READY_POSITION = 90;
    public static final int LAUNCH_POSITION = 270;

    private boolean droneLaunched = false;

    /**
     * Code to launch the paper drone.
     *
     * @param droneLauncherServo servo for drone launcher.
     */
    public DroneLauncherComponent(ServoEx droneLauncherServo) {
        droneServo = droneLauncherServo;
        droneServo.turnToAngle(READY_POSITION);
    }

    /**
     * Turns servo. (amazing)
     * Only needs to be called once, but can be called multiple times without anything happening.
     */
    public void launch() {
        if (!(droneLaunched)){
            droneServo.turnToAngle(LAUNCH_POSITION);
            droneLaunched = true;
        }
    }

    /**
     * Returns if drone has launched.
     *
     * @return if drone has launched.
     */
    public boolean getDroneLaunched(){
        return droneLaunched;
    }
}

