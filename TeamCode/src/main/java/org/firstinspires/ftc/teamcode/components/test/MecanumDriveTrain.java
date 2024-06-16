package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "Test Mecanum and Intake", group = "active-intake-test")
public class MecanumDriveTrain extends OpMode {
    private ActiveIntakeComponent activeIntake;
    private MecanumDrive drive;
    private GamepadEx player1Gamepad;

    // P1 Controls
    public static final double DEAD_ZONE_CONSTANT = 0.2;

    @Override
    public void init() {
        activeIntake = new ActiveIntakeComponent(
                new MotorEx(hardwareMap, "active_intake_motor")
        );

        Motor frontLeft = new Motor(hardwareMap, "front_left_drive");
        Motor frontRight = new Motor(hardwareMap, "front_right_drive");
        Motor backLeft = new Motor(hardwareMap, "back_left_drive");
        Motor backRight = new Motor(hardwareMap, "back_right_drive");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);


        for (Motor motor : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
        }

        player1Gamepad = new GamepadEx(gamepad1);
    }



    @Override
    public void loop() {
        if (player1Gamepad.wasJustPressed(PSButtons.SQUARE)) {
            activeIntake.turnContinually();
        }

        if (player1Gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            activeIntake.turnMotorOff();
        }

        telemetry.addLine(activeIntake.isTurning() ? "Active Intake Turning" : "Active Intake Not Turning");
        telemetry.addLine();

        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        drive.driveRobotCentric(
                Math.abs(leftX) > DEAD_ZONE_CONSTANT ? leftX : 0,
                Math.abs(leftY) > DEAD_ZONE_CONSTANT ? leftY : 0,
                Math.abs(rightX) > DEAD_ZONE_CONSTANT ? rightX : 0,
                false);

        telemetry.addData("Forward speed:", leftY);
        telemetry.addData("Turn speed:", leftX);
        telemetry.addData("Strafe speed:", rightX);

        player1Gamepad.readButtons();
    }
}