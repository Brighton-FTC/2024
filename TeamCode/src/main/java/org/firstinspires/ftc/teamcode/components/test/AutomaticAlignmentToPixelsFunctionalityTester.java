package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

/**
 * Opmode to test the functionality of the {@link AutomaticAlignmentToPixelsComponent} class. <br />
 * <p>
 * Controls:
 * <ul>
 *     <li>Dpad left/right - change the selected pixels stack. </li>
 *     <li>Cross - start/stop the robot moving.</li>
 * </ul>
 */
@TeleOp(name = "Operation Valour Functionality Tester", group = "operation-valour-test")
public class AutomaticAlignmentToPixelsFunctionalityTester extends OpMode {
    private GamepadEx gamepad;

    private ArmComponent arm;
    private ActiveIntakeComponent activeIntake;
    private MecanumDrive mecanum;
    private DistanceSensor distanceSensor;
    private IMU imu;
    private WebcamName webcam;

    private AutomaticAlignmentToPixelsComponent automaticAlignmentToPixels;

    private int pixelStackIndex = 0;

    @Override
    public void init() {
        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);

        arm = new ArmComponent(
                new MotorEx(hardwareMap, "arm_motor"),
                lynxModule.getInputVoltage(VoltageUnit.VOLTS)
        );

        activeIntake = new ActiveIntakeComponent(
                new MotorEx(hardwareMap, "active_intake_motor_left"),
                new MotorEx(hardwareMap, "active_intake_motor_right")
        );

        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "front_left_motor"),
                new Motor(hardwareMap, "front_right_motor"),
                new Motor(hardwareMap, "back_left_motor"),
                new Motor(hardwareMap, "back_right_motor")
        );

        distanceSensor = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        webcam = hardwareMap.get(WebcamName.class, "webcam");

        automaticAlignmentToPixels = new AutomaticAlignmentToPixelsComponent(
                arm, activeIntake, mecanum, distanceSensor, imu, webcam
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            pixelStackIndex = (pixelStackIndex - 1) % AutomaticAlignmentToPixelsComponent.N_PIXEL_STACKS;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            pixelStackIndex = (pixelStackIndex + 1) % AutomaticAlignmentToPixelsComponent.N_PIXEL_STACKS;
        }

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            automaticAlignmentToPixels.toggleMoving(pixelStackIndex, 2);
        }

        if (!automaticAlignmentToPixels.isMoving()) {
            telemetry.addLine("Robot moving manually. ");
            telemetry.addData("Currently selected pixel stack index", pixelStackIndex);

            mecanum.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
        } else {
            telemetry.addLine("Robot automatically aligning to pixel stack.");

            automaticAlignmentToPixels.moveRobot();
        }
    }
}
