package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.GrabberComponent;
import org.firstinspires.ftc.teamcode.components.test.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.teleop.util.PlayerButton;

import java.util.List;

public abstract class GenericTeleOp extends OpMode {

    // Override this in subclass or will crash
    public final PlayerButton DPAD_STRAFE_LEFT;
    public final PlayerButton DPAD_STRAFE_RIGHT;
    public final PlayerButton DPAD_FORWARD;
    public final PlayerButton DPAD_BACKWARDS;


    // P2 Controls
    public final PlayerButton TOGGLE_ARM_BUTTON;
    public final PlayerButton TOGGLE_GRABBER_BUTTON;
    public final PlayerButton TOGGLE_LINEAR_SLIDE_BUTTON;
    public final PlayerButton DRONE_LAUNCH_1_BUTTON;
    public final PlayerButton DRONE_LAUNCH_2_BUTTON;


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

    public static final double DEAD_ZONE_CONSTANT = 0.15;

    public static final double DPAD_DRIVE_CONSTANT = 0.75;

    /**
     * Call this in subclasses with your preferred control scheme.
     * Just copy the constructor from one of the existing subclasses.
     *
     * @param DPAD_STRAFE_LEFT Strafing left at DPAD_DRIVE_CONSTANT power to motors.
     * @param DPAD_STRAFE_RIGHT Strafing right at DPAD_DRIVE_CONSTANT power to motors.
     * @param DPAD_FORWARD Going forward at DPAD_DRIVE_CONSTANT power to motors.
     * @param DPAD_BACKWARDS Going backwards at DPAD_DRIVE_CONSTANT power to motors.
     * @param TOGGLE_ARM_BUTTON Lowers arm if lifted, lifts if not
     * @param TOGGLE_GRABBER_BUTTON Closes grabber if 
     * @param TOGGLE_LINEAR_SLIDE_BUTTON
     * @param DRONE_LAUNCH_1_BUTTON
     * @param DRONE_LAUNCH_2_BUTTON
     */
    protected GenericTeleOp(PlayerButton DPAD_STRAFE_LEFT,
                            PlayerButton DPAD_STRAFE_RIGHT,
                            PlayerButton DPAD_FORWARD,
                            PlayerButton DPAD_BACKWARDS,
                            PlayerButton TOGGLE_ARM_BUTTON,
                            PlayerButton TOGGLE_GRABBER_BUTTON,
                            PlayerButton TOGGLE_LINEAR_SLIDE_BUTTON,
                            PlayerButton DRONE_LAUNCH_1_BUTTON,
                            PlayerButton DRONE_LAUNCH_2_BUTTON){

        this.DPAD_STRAFE_LEFT = DPAD_STRAFE_LEFT;
        this.DPAD_STRAFE_RIGHT = DPAD_STRAFE_RIGHT;
        this.DPAD_FORWARD = DPAD_FORWARD;
        this.DPAD_BACKWARDS = DPAD_BACKWARDS;
        this.TOGGLE_ARM_BUTTON = TOGGLE_ARM_BUTTON ;
        this.TOGGLE_GRABBER_BUTTON = TOGGLE_GRABBER_BUTTON ;
        this.TOGGLE_LINEAR_SLIDE_BUTTON = TOGGLE_LINEAR_SLIDE_BUTTON ;
        this.DRONE_LAUNCH_1_BUTTON = DRONE_LAUNCH_1_BUTTON ;
        this.DRONE_LAUNCH_2_BUTTON = DRONE_LAUNCH_2_BUTTON;
    }

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
        linearSlide = new LinearSlideComponent(linearSlideMotor, arm);


        TOGGLE_GRABBER_BUTTON.whenPressed(grabber::toggle);
        TOGGLE_ARM_BUTTON.whenPressed(arm::toggle);
        TOGGLE_LINEAR_SLIDE_BUTTON.whenPressed(linearSlide::toggle);

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
        arm.read();

        double leftY = player1Gamepad.getLeftY();
        double leftX = player1Gamepad.getLeftX();
        double rightX = player1Gamepad.getRightX();

        leftX += DPAD_STRAFE_LEFT.isButtonPressed() ? DPAD_DRIVE_CONSTANT : 0;
        leftX -= DPAD_STRAFE_RIGHT.isButtonPressed() ? DPAD_DRIVE_CONSTANT : 0;
        leftX = Range.clip(leftX, -1, 1);

        leftY += DPAD_FORWARD.isButtonPressed() ? DPAD_DRIVE_CONSTANT : 0;
        leftY -= DPAD_BACKWARDS.isButtonPressed() ? DPAD_DRIVE_CONSTANT : 0;
        leftY = Range.clip(leftY, -1, 1);

        mecanumDrive.driveRobotCentric(
                Math.abs(leftX) > DEAD_ZONE_CONSTANT ? leftX : 0,
                Math.abs(leftY) > DEAD_ZONE_CONSTANT ? leftY : 0,
                Math.abs(rightX) > DEAD_ZONE_CONSTANT ? rightX : 0,
                false);

        if (DRONE_LAUNCH_1_BUTTON.isButtonPressed() ||
                DRONE_LAUNCH_2_BUTTON.isButtonPressed()) {
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
