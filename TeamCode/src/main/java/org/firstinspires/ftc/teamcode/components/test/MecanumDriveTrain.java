package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumDriveTrain extends OpMode {
    private Motor frontleft, frontright, backleft, backright;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    @Override
    public void init() {
        frontleft = new Motor(hardwareMap, "frontleftmotor");
        frontright = new Motor(hardwareMap, "frontrightmotor");
        backleft = new Motor(hardwareMap, "backleftmotor");
        backright = new Motor(hardwareMap, "backrightmotor");

        drive = new MecanumDrive(frontleft, frontright, backleft, backright);
        driverOp = new GamepadEx(gamepad1);
    }
    // This needs FTClib to work
    @Override
    public void loop() {
        drive.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX()
        );
    }
}
