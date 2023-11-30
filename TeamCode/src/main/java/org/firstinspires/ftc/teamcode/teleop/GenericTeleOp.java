package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;

@TeleOp(name = "TeleOp", group = "teleop-test")
public class GenericTeleOp extends OpMode {

    // P1 Controls
    public static final GamepadKeys.Button DPAD_STRAFE_LEFT = GamepadKeys.Button.DPAD_LEFT;
    public static final GamepadKeys.Button DPAD_STRAFE_RIGHT = GamepadKeys.Button.DPAD_RIGHT;
    public static final GamepadKeys.Button DPAD_FORWARD = GamepadKeys.Button.DPAD_UP;
    public static final GamepadKeys.Button DPAD_BACKWARDS = GamepadKeys.Button.DPAD_DOWN;


    // P2 Controls
    public static final GamepadKeys.Button TOGGLE_ARM_BUTTON = PSButtons.CROSS;
    public static final GamepadKeys.Button TOGGLE_GRABBER_BUTTON = PSButtons.CIRCLE;
    public static final GamepadKeys.Button DRONE_LAUNCH_1_BUTTON = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button DRONE_LAUNCH_2_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;




    private GamepadEx player1Gamepad;
    private GamepadEx player2Gamepad;

    private ServoEx grabberServo1;
    private ServoEx grabberServo2;
    private GrabberComponent grabber;

    private MotorEx armMotor;
    private ServoEx grabberRotatorServo;
    private ArmComponent arm;

    private ServoEx droneServo;
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

        droneServo = new SimpleServo(hardwareMap, "drone_servo",
                DroneLauncherComponent.READY_POSITION,
                DroneLauncherComponent.LAUNCH_POSITION);

        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeftDrive"),
                new Motor(hardwareMap, "frontRightDrive"),
                new Motor(hardwareMap, "backLeftDrive"),
                new Motor(hardwareMap, "backRightDrive"));


        arm = new ArmComponent(armMotor, grabberRotatorServo);
        grabber = new GrabberComponent(grabberServo1, grabberServo2);
        droneLauncher = new DroneLauncherComponent(droneServo);


        player2Gamepad.getGamepadButton(TOGGLE_GRABBER_BUTTON).whenPressed(grabber::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_ARM_BUTTON).whenPressed(arm::toggle);
    }

    @Override
    public void loop() {
        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        if (player1Gamepad.getButton(DPAD_STRAFE_LEFT)){
            leftX += 0.75;
        }
        if (player1Gamepad.getButton(DPAD_STRAFE_RIGHT)){
            leftX -= 0.75;
        }
        leftX = Range.clip(leftX, -1, 1);

        if (player1Gamepad.getButton(DPAD_FORWARD)){
            leftY += 0.75;
        }
        if (player1Gamepad.getButton(DPAD_BACKWARDS)){
            leftY -= 0.75;
        }
        leftY = Range.clip(leftY, -1, 1);

        mecanumDrive.driveRobotCentric(leftX, leftY, rightX,false);

        if (player1Gamepad.getButton(DRONE_LAUNCH_1_BUTTON) || player1Gamepad.getButton(DRONE_LAUNCH_2_BUTTON)){
            droneLauncher.launch();
        }

        telemetry.update();
    }
}
