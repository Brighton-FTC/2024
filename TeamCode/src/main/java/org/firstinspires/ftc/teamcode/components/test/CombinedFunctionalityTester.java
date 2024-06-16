package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "Combined Functionality Tester", group = "test")
public class CombinedFunctionalityTester extends OpMode {
    private ArmComponent arm;
    private MotorEx intakeMotor;
    private MecanumDrive mecanumDrive;
    private ServoEx frontOuttakeServo, backOuttakeServo;

    private GamepadEx gamepad;

    private IMU imu;
    private boolean isIntakeTurning = false;

    @Override
    public void init() {
        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        arm.getArmMotor().resetEncoder();

        intakeMotor = new MotorEx(hardwareMap, "active_intake_motor");

        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        );

        frontOuttakeServo = new SimpleServo(hardwareMap, "outtake_servo_front", 0, 90);
        backOuttakeServo = new SimpleServo(hardwareMap, "outtake_servo_back", 0, 90);

        gamepad = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        imu.resetYaw();
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        arm.read();

        mecanumDrive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            arm.setState(ArmComponent.State.PICKUP_GROUND);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (arm.getState() == ArmComponent.State.PLACE_HIGH_BACKDROP) {
                arm.setState(ArmComponent.State.PLACE_LOW_BACKDROP);
            } else {
                arm.setState(ArmComponent.State.PLACE_HIGH_BACKDROP);
            }
        }
//        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            if (arm.getState() == ArmComponent.State.PLACE_BACKDROP) {
//                arm.setState(ArmComponent.State.PLACE_GROUND);
//            } else {
//                arm.setState(ArmComponent.State.PLACE_BACKDROP);
//            }
//        }

        arm.moveToSetPoint();

        if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            if (isIntakeTurning) {
                intakeMotor.set(0);
            } else {
                intakeMotor.set(-1);
            }
            isIntakeTurning = !isIntakeTurning;
        }

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            if (frontOuttakeServo.getPosition() == 0) {
                frontOuttakeServo.turnToAngle(90);
            } else {
                frontOuttakeServo.turnToAngle(0);
            }
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            if (backOuttakeServo.getPosition() == 0) {
                backOuttakeServo.turnToAngle(90);
            } else {
                backOuttakeServo.turnToAngle(0);
            }
        }
    }
}
