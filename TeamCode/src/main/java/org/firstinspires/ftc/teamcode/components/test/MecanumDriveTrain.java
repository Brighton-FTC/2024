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
    public static final GamepadKeys.Button DPAD_STRAFE_LEFT = GamepadKeys.Button.DPAD_LEFT;
    public static final GamepadKeys.Button DPAD_STRAFE_RIGHT = GamepadKeys.Button.DPAD_RIGHT;
    public static final GamepadKeys.Button DPAD_FORWARD = GamepadKeys.Button.DPAD_UP;
    public static final GamepadKeys.Button DPAD_BACKWARDS = GamepadKeys.Button.DPAD_DOWN;
    public static final double DEAD_ZONE_CONSTANT = 0.2;

    @Override
    public void init() {
        activeIntake = new ActiveIntakeComponent(
                new MotorEx(hardwareMap, "active_intake_motor_left"),
                new MotorEx(hardwareMap, "active_intake_motor_right")
        );

        Motor frontleft = new Motor(hardwareMap, "front_left_drive");
        Motor frontright = new Motor(hardwareMap, "front_right_drive");
        Motor backleft = new Motor(hardwareMap, "back_left_drive");
        Motor backright = new Motor(hardwareMap, "back_right_drive");

        drive = new MecanumDrive(frontleft, frontright, backleft, backright);


        for (Motor motor : new Motor[]{frontleft, frontright, backleft, backright}) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setInverted(!motor.getInverted());
        }

        player1Gamepad = new GamepadEx(gamepad1);
    }



    @Override
    public void loop() {
        if (player1Gamepad.wasJustPressed(PSButtons.CROSS)) {
            activeIntake.turnManually();
        }
        if (player1Gamepad.wasJustPressed(PSButtons.SQUARE)) {
            activeIntake.turnContinually();
        }

        if (player1Gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            activeIntake.turnMotorOff();
        }

        telemetry.addData("State:", activeIntake.getState().toString());
        telemetry.addData("Motor 1 position", activeIntake.getMotors()[0]);
        telemetry.addData("Motor 2 position", activeIntake.getMotors()[1]);
        telemetry.addLine();

        activeIntake.moveMotor();
        double leftY = -player1Gamepad.getLeftY();
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

        telemetry.addData("Forward speed:", leftY);
        telemetry.addData("Turn speed:", leftX);
        telemetry.addData("Strafe speed:", rightX);

        player1Gamepad.readButtons();
    }
}
