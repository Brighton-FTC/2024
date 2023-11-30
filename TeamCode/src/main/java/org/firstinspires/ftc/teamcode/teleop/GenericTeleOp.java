package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;

public class GenericTeleOp extends OpMode {

    // P1 Controls

//    public static final GamepadKeys.Button OPERATION_VALOUR_BUTTON = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button OPERATION_VALOUR_BUTTON = PSButtons.CROSS;

    // P2 Controls
    public static final GamepadKeys.Button TOGGLE_ARM_BUTTON = PSButtons.CROSS;
    public static final GamepadKeys.Button TOGGLE_GRABBER_BUTTON = PSButtons.CIRCLE;
    public static final GamepadKeys.Button DRONE_LAUNCH_BUTTON = PSButtons.SQUARE;


    private GamepadEx player1Gamepad;
    private GamepadEx player2Gamepad;

    private ServoEx grabberServo1;
    private ServoEx grabberServo2;
    private GrabberComponent grabber;

    private MotorEx armMotor;
    private ServoEx grabberRotatorServo;
    private ArmComponent arm;

    private MotorEx droneMotor;
    private DroneLauncherComponent droneLauncher;

    private MecanumDrive mecanumDrive;


    @Override
    public void init() {
        player1Gamepad = new GamepadEx(gamepad1);
        player2Gamepad = new GamepadEx(gamepad2);

        grabberServo1 = new SimpleServo(hardwareMap, "grabber_servo",
                GrabberComponent.GRABBER_CLOSED_POSITION,
                GrabberComponent.GRABBER_OPEN_POSITION);
        grabberServo2 = new SimpleServo(hardwareMap, "grabber_servo",
                GrabberComponent.GRABBER_CLOSED_POSITION,
                GrabberComponent.GRABBER_OPEN_POSITION);

        armMotor = new MotorEx(hardwareMap, "arm_motor");
        grabberRotatorServo = new SimpleServo(hardwareMap, "grabber_rotator_servo",
                ArmComponent.GRABBER_TILT_DOWN_POSITION,
                ArmComponent.GRABBER_TILT_UP_POSITION);

        droneLauncher = new DroneLauncherComponent()

        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeftDrive"),
                new Motor(hardwareMap, "frontRightDrive"),
                new Motor(hardwareMap, "backLeftDrive"),
                new Motor(hardwareMap, "backRightDrive"));



        arm = new ArmComponent(armMotor, grabberRotatorServo);
        grabber = new GrabberComponent(grabberServo1, grabberServo2);


        player2Gamepad.getGamepadButton(TOGGLE_GRABBER_BUTTON).whenPressed(grabber::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_ARM_BUTTON).whenPressed(arm::toggle);
        player2Gamepad.getGamepadButton(DRONE_LAUNCH_BUTTON).whenPressed(arm::toggle);
    }

    @Override
    public void loop() {
        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        mecanumDrive.driveRobotCentric(leftX, leftY, rightX,false);

        telemetry.update();
    }
}
