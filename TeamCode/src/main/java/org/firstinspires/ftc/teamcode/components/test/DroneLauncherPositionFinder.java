package org.firstinspires.ftc.teamcode.components.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Some code I made to get servo for drone launcher.
 * Controls:
 * <ul>
 *     <li>Rotate servo - dpad left & dpad right</li>
 *     <li>Change rotation angle - dpad down & dpad up</li>
 * </ul>
 */

@Config
@TeleOp(name = "Drone Launcher Position Tester", group = "drone-test")
public class DroneLauncherPositionFinder extends OpMode {
    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    private ServoEx droneLauncherServo;

    public static int rotationAngle = 20;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        droneLauncherServo = new SimpleServo(hardwareMap, "drone_servo", 0, 360);
    }

    @Override
    public void loop() {
        droneLauncherServo.turnToAngle(rotationAngle);

        telemetry.addData("Rotation angle", rotationAngle);
        telemetry.addData("Motor position", droneLauncherServo.getAngle());
        telemetry.update();
    }
}