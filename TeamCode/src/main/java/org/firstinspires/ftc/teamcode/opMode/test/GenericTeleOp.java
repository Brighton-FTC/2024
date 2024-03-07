package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.HeadingPID;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.teleop.util.PlayerButton;
import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;

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
    public PlayerButton OUTTAKE_RELEASE_ONE_PIXEL;
    public PlayerButton DRONE_LEFT_RELEASE;

    private HeadingPID headingPID;

    private BCGyro gyro;

    private ElapsedTime time;

    public GenericTeleOp(){
    }
    protected void setButtons(
        PlayerButton DRIVETRAIN_SLOW_MODE,
        PlayerButton ARM_STATE_FORWARD,
        PlayerButton ARM_STATE_BACKWARDS,
        PlayerButton ARM_STATE_DOWN,
        PlayerButton TURN_INTAKE_CONSTANT,
        PlayerButton TURN_INTAKE_MANUAL,
        PlayerButton OUTTAKE_RELEASE_ALL_PIXEL,
        PlayerButton OUTTAKE_RELEASE_ONE_PIXEL,
        PlayerButton DRONE_LEFT_RELEASE
    ){
        this.DRIVETRAIN_SLOW_MODE = DRIVETRAIN_SLOW_MODE;
        this.ARM_STATE_FORWARD = ARM_STATE_FORWARD;
        this.ARM_STATE_BACKWARDS = ARM_STATE_BACKWARDS;
        this.ARM_STATE_DOWN = ARM_STATE_DOWN;
        this.TURN_INTAKE_CONSTANT = TURN_INTAKE_CONSTANT;
        this.TURN_INTAKE_MANUAL = TURN_INTAKE_MANUAL;
        this.OUTTAKE_RELEASE_ALL_PIXEL = OUTTAKE_RELEASE_ALL_PIXEL;
        this.OUTTAKE_RELEASE_ONE_PIXEL = OUTTAKE_RELEASE_ONE_PIXEL;
        this.DRONE_LEFT_RELEASE = DRONE_LEFT_RELEASE;
    }

    @Override
    public void init() {
        gamepadp1 = new GamepadEx(gamepad1);

        gyro = new BCGyro(hardwareMap);
        gyro.init();

        headingPID = new HeadingPID(gyro);

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);
        lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"), lynxModule.getInputVoltage(VoltageUnit.VOLTS));
        droneLauncher = new DroneLauncherComponent(new SimpleServo(hardwareMap, "drone_launcher_servo", 0, 360));
        activeIntake = new ActiveIntakeComponent(new MotorEx(hardwareMap, "active_intake_motor_left"), new MotorEx(hardwareMap, "active_intake_motor_right"));
        outtake = new OuttakeComponent(new SimpleServo(hardwareMap, "outtake_servo", 0, 360));

        Motor[] driveMotors = {
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        };

        for (Motor motor : driveMotors) {
            motor.setInverted(true);
            motor.setRunMode(Motor.RunMode.VelocityControl);
        }

        driveMotors[1].setInverted(false); // un-invert front right, because hardware is weird.

        mecanum = new MecanumDrive(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3]);
    }

    @Override
    public void start() {
        // prank: uncomment if wanted
        /*
        gamepad1.rumble(Integer.MAX_VALUE);
        gamepad1.runLedEffect(new Gamepad.LedEffect.Builder()
                .addStep(255, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .setRepeating(true)
                .build());
         */
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

        if (arm.getState() == ArmComponent.State.GROUND && arm.atSetPoint()) {
            activeIntake.moveMotor();
        }

        if (ARM_STATE_FORWARD.wasJustPressed()) {
            int newIndex = selectedState.index+1 <= ArmComponent.State.values().length ?
                    selectedState.index+1 : 1;
            selectedState = ArmComponent.State.values()[newIndex];
        }

        if (ARM_STATE_BACKWARDS.wasJustPressed()) {
            int newIndex = selectedState.index-1 >= 0 ?
                    selectedState.index-1 : 3;
            selectedState = ArmComponent.State.values()[newIndex];
        }

        arm.setState(selectedState);

        if (ARM_STATE_DOWN.wasJustPressed()) {
            arm.setState(ArmComponent.State.GROUND);
        }

        if (TURN_INTAKE_CONSTANT.wasJustPressed()) {
            if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                arm.setState(ArmComponent.State.GROUND);
                activeIntake.turnContinually();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        if (TURN_INTAKE_MANUAL.wasJustPressed()) {
            activeIntake.turnManually();
        }

        if (OUTTAKE_RELEASE_ONE_PIXEL.wasJustPressed()) {
            outtake.releasePixel();
        }

        if (OUTTAKE_RELEASE_ALL_PIXEL.wasJustPressed()) {
            outtake.releaseAllPixels();
        }

        if (DRONE_LEFT_RELEASE.wasJustPressed()) {
            droneLauncher.launch();
        }

        telemetry.addData("Arm state", arm.getState());
        telemetry.addData("Selected arm state", selectedState);
        telemetry.addData("Active intake state", activeIntake.getState());
        telemetry.addLine(outtake.isClosed() ? "Outtake closed" : "Outtake open");
        telemetry.addLine(droneLauncher.getDroneLaunched() ? "Drone launched" : "Drone not launched");
    }
}
