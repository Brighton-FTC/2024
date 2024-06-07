package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.components.test.heading.HeadingPID;
import org.firstinspires.ftc.teamcode.util.teleop.PlayerButton;

/**
 * teleop
 */
public abstract class GenericTeleOp extends OpMode {
    protected GamepadEx gamepadp1;
    protected GamepadEx gamepadp2;

    private ArmComponent arm;
    private DroneLauncherComponent droneLauncher;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    private MecanumDrive mecanum;

    private IMU imu;

    private final double NORMAL_DRIVE_MULTIPLIER = 1;
    private final double SLOW_DRIVE_MULTIPLIER = 0.25;
    private final double DEAD_ZONE_SIZE = 0.2;
    private boolean isSlowMode;
    private ArmComponent.State selectedState;

    public PlayerButton DRIVETRAIN_SLOW_MODE;
    public PlayerButton ARM_STATE_FORWARD;
    public PlayerButton ARM_STATE_BACKWARDS;
    public PlayerButton ARM_STATE_DOWN;
    public PlayerButton TURN_INTAKE_CONSTANT;
    public PlayerButton TURN_INTAKE_MANUAL;
    public PlayerButton OUTTAKE_RELEASE_ALL_PIXEL;
    public PlayerButton DRONE_LEFT_RELEASE;

    private HeadingPID headingPID;

    private ElapsedTime time;

    public GenericTeleOp() {
    }

    protected void setButtons(
            PlayerButton DRIVETRAIN_SLOW_MODE,
            PlayerButton ARM_STATE_FORWARD,
            PlayerButton ARM_STATE_BACKWARDS,
            PlayerButton ARM_STATE_DOWN,
            PlayerButton TURN_INTAKE_CONSTANT,
            PlayerButton TURN_INTAKE_MANUAL,
            PlayerButton OUTTAKE_RELEASE_ALL_PIXEL,
            PlayerButton DRONE_LEFT_RELEASE
    ) {
        this.DRIVETRAIN_SLOW_MODE = DRIVETRAIN_SLOW_MODE;
        this.ARM_STATE_FORWARD = ARM_STATE_FORWARD;
        this.ARM_STATE_BACKWARDS = ARM_STATE_BACKWARDS;
        this.ARM_STATE_DOWN = ARM_STATE_DOWN;
        this.TURN_INTAKE_CONSTANT = TURN_INTAKE_CONSTANT;
        this.TURN_INTAKE_MANUAL = TURN_INTAKE_MANUAL;
        this.OUTTAKE_RELEASE_ALL_PIXEL = OUTTAKE_RELEASE_ALL_PIXEL;
        this.DRONE_LEFT_RELEASE = DRONE_LEFT_RELEASE;
    }

    @Override
    public void init() {
        gamepadp1 = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");

        // TODO: check if these are correct
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        headingPID = new HeadingPID(imu);

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);
        lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        droneLauncher = new DroneLauncherComponent(new SimpleServo(hardwareMap, "drone_launcher_servo", 0, 360));
        activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "active_intake_motor"));
        outtake = new OuttakeComponent(
                new SimpleServo(hardwareMap, "outtake_servo_front", 0, 360),
                new SimpleServo(hardwareMap, "outtake_servo_back", 0, 360)
        );

        Motor[] driveMotors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        for (Motor motor : driveMotors) {
            motor.setInverted(true); // TODO: check if inverting these is correct for current drivetrain.
            motor.setRunMode(Motor.RunMode.VelocityControl);
        }

        mecanum = new MecanumDrive(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3]);
    }

    @Override
    public void loop() {
        gamepadp1.readButtons();
        gamepadp2.readButtons();

        // drivetrain
        if (DRIVETRAIN_SLOW_MODE.wasJustPressed()) {
            isSlowMode = !isSlowMode;
        }

        double headingCorrection = headingPID.runPID(gamepadp1.getRightX(), time.milliseconds());

        time.reset();

        double[] driveCoefficients = {
                gamepadp1.getLeftX(),
                gamepadp1.getLeftY(),
                gamepadp1.getRightX() + headingCorrection
        };
        for (int i = 0; i < driveCoefficients.length; i++) {
            driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
            driveCoefficients[i] = isSlowMode ? driveCoefficients[i] * SLOW_DRIVE_MULTIPLIER : driveCoefficients[i] * NORMAL_DRIVE_MULTIPLIER;
            driveCoefficients[i] = Range.clip(-1, 1, driveCoefficients[i]); // clip in case of multiplier that is greater than 1
        }
        mecanum.driveRobotCentric(driveCoefficients[0], driveCoefficients[1], driveCoefficients[2]);

        telemetry.addData("Forward speed", driveCoefficients[1]);
        telemetry.addData("Strafe speed", driveCoefficients[0]);
        telemetry.addData("Turn speed", driveCoefficients[2]);
        telemetry.addLine(isSlowMode ? "Slow mode activated" : "Normal speed");
        telemetry.addLine();

        // components
        arm.read();
        arm.moveToSetPoint();

        if (arm.getState() == ArmComponent.State.PICKUP_GROUND && arm.atSetPoint()) {
            if (!activeIntake.isTurning()) {
                activeIntake.turnContinually();
            }
        } else {
            activeIntake.turnMotorOff();
        }

        if (ARM_STATE_FORWARD.wasJustPressed()) {
            selectedState = selectedState.next();
        }

        if (ARM_STATE_BACKWARDS.wasJustPressed()) {
            selectedState = selectedState.previous();
        }

        arm.setState(selectedState);

        if (ARM_STATE_DOWN.wasJustPressed()) {
            arm.setState(ArmComponent.State.PICKUP_GROUND);
        }

        if (OUTTAKE_RELEASE_ALL_PIXEL.wasJustPressed()) {
            outtake.releaseAll();
        }

        if (DRONE_LEFT_RELEASE.wasJustPressed()) {
            droneLauncher.launch();
        }

        telemetry.addData("Arm state", arm.getState());
        telemetry.addData("Selected arm state", selectedState);
        telemetry.addLine(activeIntake.isTurning() ? "Active Intake Turning" : "Active Intake not Turning");
        telemetry.addLine(outtake.areServosTurned().first ? "Outtake Front Turned" : "Outtake Front Not Turned");
        telemetry.addLine(outtake.areServosTurned().first ? "Outtake Back Turned" : "Outtake Back Not Turned");
        telemetry.addLine(droneLauncher.getDroneLaunched() ? "Drone launched" : "Drone not launched");
    }
}
