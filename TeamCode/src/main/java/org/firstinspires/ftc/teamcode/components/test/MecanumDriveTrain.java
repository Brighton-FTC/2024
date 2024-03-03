package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Mecanum", group = "test")
public class MecanumDriveTrain extends OpMode {
    private Motor frontleft, frontright, backleft, backright;
    private MecanumDrive drive;
    private GamepadEx player1Gamepad;

    // P1 Controls
    public static final GamepadKeys.Button DPAD_STRAFE_LEFT = GamepadKeys.Button.DPAD_LEFT;
    public static final GamepadKeys.Button DPAD_STRAFE_RIGHT = GamepadKeys.Button.DPAD_RIGHT;
    public static final GamepadKeys.Button DPAD_FORWARD = GamepadKeys.Button.DPAD_UP;
    public static final GamepadKeys.Button DPAD_BACKWARDS = GamepadKeys.Button.DPAD_DOWN;
    public static final double DEAD_ZONE_CONSTANT = 0.2;

    @Override
    public void init() {
        frontleft = new Motor(hardwareMap, "front_left_drive");
        frontright = new Motor(hardwareMap, "front_right_drive");
        backleft = new Motor(hardwareMap, "back_left_drive");
        backright = new Motor(hardwareMap, "back_right_drive");

        drive = new MecanumDrive(frontleft, frontright, backleft, backright);

        for (Motor motor : new Motor[]{frontleft, backleft, backright}) {
            motor.setInverted(!motor.getInverted());
        }

        player1Gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        double leftY = player1Gamepad.getLeftY();
        double leftX = -player1Gamepad.getLeftX();
        double rightX = -player1Gamepad.getRightX();

        leftX += player1Gamepad.getButton(DPAD_STRAFE_LEFT) ? 0.75 : 0;
        leftX -= player1Gamepad.getButton(DPAD_STRAFE_RIGHT) ? 0.75 : 0;
        leftX = Range.clip(leftX, -1, 1);

        leftY += player1Gamepad.getButton(DPAD_FORWARD) ? 0.75 : 0;
        leftY -= player1Gamepad.getButton(DPAD_BACKWARDS) ? 0.75 : 0;
        leftY = Range.clip(leftY, -1, 1);

        drive.driveRobotCentric(
                Math.abs(leftX) > DEAD_ZONE_CONSTANT ? leftX : 0,
                Math.abs(leftY) > DEAD_ZONE_CONSTANT ? leftY : 0,
                Math.abs(rightX) > DEAD_ZONE_CONSTANT ? rightX : 0,
                false);
    }
}
