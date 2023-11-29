package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * Opmode to test the functionality of the {@link AutomaticAlignmentToPixelsComponent} class. <br />
 *
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
    private LinearSlideComponent linearSlide;
    private GrabberComponent grabber;
    private MecanumDrive mecanum;
    private SensorDistanceEx distanceSensor;
    private GyroEx gyro;
    private WebcamName webcam;

    private AutomaticAlignmentToPixelsComponent automaticAlignmentToPixels;

    private int pixelStackIndex = 0;

    @Override
    public void init() {
        arm = new ArmComponent(
                new MotorEx(hardwareMap, "arm_motor"),
                new SimpleServo(hardwareMap, "grabber_rotation_servo", 0, 360)
        );

        linearSlide = new LinearSlideComponent(
                new MotorEx(hardwareMap, "linear_slide_motor")
        );

        grabber = new GrabberComponent(
                new SimpleServo(hardwareMap, "grabber_servo_1", 0, 360),
                new SimpleServo(hardwareMap, "grabber_servo_2", 0, 360),
                hardwareMap.touchSensor.get("touch_sensor")
        );

        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "front_left_motor"),
                new Motor(hardwareMap, "front_right_motor"),
                new Motor(hardwareMap, "back_left_motor"),
                new Motor(hardwareMap, "back_right_motor")
        );

        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance_sensor");

        gyro = new RevIMU(hardwareMap, "gyro");

        webcam = hardwareMap.get(WebcamName.class, "webcam");

        automaticAlignmentToPixels = new AutomaticAlignmentToPixelsComponent(
                arm, linearSlide, grabber, mecanum, distanceSensor, gyro, webcam
        );

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> pixelStackIndex = (pixelStackIndex - 1) % AutomaticAlignmentToPixelsComponent.N_PIXEL_STACKS);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> pixelStackIndex = (pixelStackIndex + 1) % AutomaticAlignmentToPixelsComponent.N_PIXEL_STACKS);

        gamepad.getGamepadButton(PSButtons.CROSS).whenPressed(() -> automaticAlignmentToPixels.toggleMoving(pixelStackIndex));
    }
    @Override
    public void loop() {
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
