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
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;

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
    public static final GamepadKeys.Button TOGGLE_LINEAR_SLIDE_BUTTON = PSButtons.SQUARE;
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

    private LinearSlideComponent linearSlide;
    private MotorEx linearSlideMotor;

    private MecanumDrive mecanumDrive;


    @Override
    public void init() {
        player1Gamepad = new GamepadEx(gamepad1);
        player2Gamepad = new GamepadEx(gamepad2);

        grabberServo1 = new SimpleServo(hardwareMap, "grabber_servo_1",
                GrabberComponent.GRABBER_CLOSED_POSITION,
                GrabberComponent.GRABBER_OPEN_POSITION);
        grabberServo2 = new SimpleServo(hardwareMap, "grabber_servo_2",
                GrabberComponent.GRABBER_CLOSED_POSITION,
                GrabberComponent.GRABBER_OPEN_POSITION);

        armMotor = new MotorEx(hardwareMap, "arm_motor");
        grabberRotatorServo = new SimpleServo(hardwareMap, "grabber_rotator_servo",
                ArmComponent.GRABBER_TILT_DOWN_POSITION,
                ArmComponent.GRABBER_TILT_UP_POSITION);

        droneServo = new SimpleServo(hardwareMap, "drone_servo",
                DroneLauncherComponent.READY_POSITION,
                DroneLauncherComponent.LAUNCH_POSITION);

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_motor");

        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeftDrive"),
                new Motor(hardwareMap, "frontRightDrive"),
                new Motor(hardwareMap, "backLeftDrive"),
                new Motor(hardwareMap, "backRightDrive"));


        arm = new ArmComponent(armMotor, grabberRotatorServo);
        grabber = new GrabberComponent(grabberServo1, grabberServo2);
        droneLauncher = new DroneLauncherComponent(droneServo);
        linearSlide = new LinearSlideComponent(linearSlideMotor);


        player2Gamepad.getGamepadButton(TOGGLE_GRABBER_BUTTON).whenPressed(grabber::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_ARM_BUTTON).whenPressed(arm::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_LINEAR_SLIDE_BUTTON).whenPressed(linearSlide::toggle);
    }

    @Override
    public void loop() {
        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        leftX += player1Gamepad.getButton(DPAD_STRAFE_LEFT) ? 0.75 : 0;
        leftX -= player1Gamepad.getButton(DPAD_STRAFE_RIGHT) ? 0.75 : 0;
        leftX = Range.clip(leftX, -1, 1);

        leftY += player1Gamepad.getButton(DPAD_FORWARD) ? 0.75 : 0;
        leftY -= player1Gamepad.getButton(DPAD_BACKWARDS) ? 0.75 : 0;
        leftY = Range.clip(leftY, -1, 1);

        mecanumDrive.driveRobotCentric(leftX, leftY, rightX,false);

        if (player1Gamepad.getButton(DRONE_LAUNCH_1_BUTTON) || player1Gamepad.getButton(DRONE_LAUNCH_2_BUTTON)){
            droneLauncher.launch();
        }

        arm.moveToSetPosition();
        linearSlide.moveToSetPoint();

        telemetry.addData("Arm position", arm.getArmPosition());
        telemetry.addData("Grabber closed?", grabber.isClosed());
        telemetry.addData("Linear slide position", linearSlide.getPosition());
        telemetry.update();
    }
}
