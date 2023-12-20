package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.inputs.PSButtons;

import java.util.List;

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
    private ArmComponent arm;

    private ServoEx droneServo;
    private DroneLauncherComponent droneLauncher;

    private LinearSlideComponent linearSlide;
    private MotorEx linearSlideMotor;

    private MecanumDrive mecanumDrive;

    private ElapsedTime time1;
    private ElapsedTime time2;

    private List<LynxModule> allHubs;

    public static final double DEAD_ZONE_CONSTANT = 0.05;


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

        droneServo = new SimpleServo(hardwareMap, "drone_servo",
                DroneLauncherComponent.READY_POSITION,
                DroneLauncherComponent.LAUNCH_POSITION);

        linearSlideMotor = new MotorEx(hardwareMap, "linear_slide_motor");

        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeftDrive"),
                new Motor(hardwareMap, "frontRightDrive"),
                new Motor(hardwareMap, "backLeftDrive"),
                new Motor(hardwareMap, "backRightDrive"));


        arm = new ArmComponent(armMotor);
        grabber = new GrabberComponent(grabberServo1, grabberServo2);
        droneLauncher = new DroneLauncherComponent(droneServo);
        linearSlide = new LinearSlideComponent(linearSlideMotor);


        player2Gamepad.getGamepadButton(TOGGLE_GRABBER_BUTTON).whenPressed(grabber::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_ARM_BUTTON).whenPressed(arm::toggle);
        player2Gamepad.getGamepadButton(TOGGLE_LINEAR_SLIDE_BUTTON).whenPressed(linearSlide::toggle);
        time1 = new ElapsedTime();
        time2 = new ElapsedTime();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        linearSlide.read();

        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        leftX += player1Gamepad.getButton(DPAD_STRAFE_LEFT) ? 0.75 : 0;
        leftX -= player1Gamepad.getButton(DPAD_STRAFE_RIGHT) ? 0.75 : 0;
        leftX = Range.clip(leftX, -1, 1);

        leftY += player1Gamepad.getButton(DPAD_FORWARD) ? 0.75 : 0;
        leftY -= player1Gamepad.getButton(DPAD_BACKWARDS) ? 0.75 : 0;
        leftY = Range.clip(leftY, -1, 1);

        mecanumDrive.driveRobotCentric(
                Math.abs(leftX) > DEAD_ZONE_CONSTANT ? leftX : 0,
                Math.abs(leftY) > DEAD_ZONE_CONSTANT ? leftY : 0,
                Math.abs(rightX) > DEAD_ZONE_CONSTANT ? rightX : 0,
                false);

        if (player1Gamepad.getButton(DRONE_LAUNCH_1_BUTTON) || player1Gamepad.getButton(DRONE_LAUNCH_2_BUTTON)) {
            droneLauncher.launch();
        }

        // prank lol lmao
        // saku don't remove this thanks
        // why is the ide showing your name as a error
        if (time1.seconds() > 20){
            gamepad1.rumble(1, 1, 100);
            gamepad2.rumble(1, 1, 100);

            if (time2.seconds() > 1) {
                Gamepad.LedEffect effect = new Gamepad.LedEffect.Builder()
                        .addStep(1, 0, 0, 100)
                        .addStep(0, 1, 0, 100)
                        .addStep(0, 0, 1, 100)
                        .addStep(1, 1, 1, 100)
                        .addStep(1, 0, 0, 100)
                        .addStep(1, 1, 0, 100)
                        .addStep(0, 1, 1, 100)
                        .addStep(1, 0, 1, 100)
                        .addStep(0, 0, 0, 100)
                        .addStep(1, 1, 0, 100)
                        .build();
                gamepad1.runLedEffect(effect);
                gamepad2.runLedEffect(effect);
                time2.reset();
            }
        }

        arm.moveToSetPoint();
        linearSlide.moveToSetPoint();

        player1Gamepad.readButtons();
        player2Gamepad.readButtons();

        telemetry.addData("Arm position", arm.getArmPosition());
        telemetry.addData("Grabber closed?", grabber.isClosed());
        telemetry.addData("Linear slide position", linearSlide.getPosition());
        telemetry.update();
    }
}
