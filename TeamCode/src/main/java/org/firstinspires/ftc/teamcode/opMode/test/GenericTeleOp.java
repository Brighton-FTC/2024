package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
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
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.util.teleop.PlayerButton;

import java.util.List;

/**
 * teleop
 */
public abstract class GenericTeleOp extends OpMode {
    protected GamepadEx gamepadp1;
    protected GamepadEx gamepadp2;

    private ArmComponent arm;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    private MecanumDrive mecanum;

    private IMU imu;

    private final double NORMAL_DRIVE_MULTIPLIER = 1;
    private final double SLOW_DRIVE_MULTIPLIER = 0.25;
    private final double DEAD_ZONE_SIZE = 0.2;
    private boolean isSlowMode;

    public PlayerButton DRIVETRAIN_SLOW_MODE;
    public PlayerButton ARM_STATE_FORWARD;
    public PlayerButton ARM_STATE_DOWN;

    public PlayerButton TURN_INTAKE_FORWARDS;
    public PlayerButton TURN_INTAKE_BACKWARDS;
    public PlayerButton OUTTAKE_TOGGLE_ALL_PIXEL;
    public PlayerButton OUTTAKE_TOGGLE_BACK_PIXEL;
    public PlayerButton DRONE_LEFT_RELEASE;

    private ElapsedTime time;

    private ServoEx droneServo;

    private List<LynxModule> allHubs;

    public GenericTeleOp() {
    }

    protected void setButtons(
            PlayerButton DRIVETRAIN_SLOW_MODE,
            PlayerButton ARM_STATE_FORWARD,
            PlayerButton ARM_STATE_DOWN,
            PlayerButton TURN_INTAKE_FORWARDS,
            PlayerButton TURN_INTAKE_BACKWARDS,
            PlayerButton OUTTAKE_TOGGLE_ALL_PIXEL,
            PlayerButton OUTTAKE_TOGGLE_BACK_PIXEL,
            PlayerButton DRONE_LEFT_RELEASE
    ) {
        this.DRIVETRAIN_SLOW_MODE = DRIVETRAIN_SLOW_MODE;
        this.ARM_STATE_FORWARD = ARM_STATE_FORWARD;
        this.ARM_STATE_DOWN = ARM_STATE_DOWN;
        this.TURN_INTAKE_FORWARDS = TURN_INTAKE_FORWARDS;
        this.TURN_INTAKE_BACKWARDS = TURN_INTAKE_BACKWARDS;
        this.OUTTAKE_TOGGLE_ALL_PIXEL = OUTTAKE_TOGGLE_ALL_PIXEL;
        this.OUTTAKE_TOGGLE_BACK_PIXEL = OUTTAKE_TOGGLE_BACK_PIXEL;
        this.DRONE_LEFT_RELEASE = DRONE_LEFT_RELEASE;
    }
        @Override
        public void init() {
            gamepadp1 = new GamepadEx(gamepad1);
            gamepadp2 = new GamepadEx(gamepad2);

            imu = hardwareMap.get(IMU.class, "imu");

            // TODO: check if these are correct
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            ));
            allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
        arm.getArmMotor().resetEncoder();
        droneServo = new SimpleServo(hardwareMap, "drone_launcher_servo", 0, 360);
        droneServo.turnToAngle(0);
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
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
        gamepadp1.readButtons();
        gamepadp2.readButtons();

        // --- DRIVETRAIN ---

        if (DRIVETRAIN_SLOW_MODE.wasJustPressed()) {
            isSlowMode = !isSlowMode;
        }

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        time.reset();

        double[] driveCoefficients = {
                gamepadp1.getLeftX(),
                gamepadp1.getLeftY(),
                gamepadp1.getRightX()
        };
        for (int i = 0; i < driveCoefficients.length; i++) {
            driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
            driveCoefficients[i] = Range.clip(driveCoefficients[i], -1, 1); // clip in case of multiplier that is greater than 1
            driveCoefficients[i] = (Math.pow(driveCoefficients[i], 3) + driveCoefficients[i]) / 2;
            driveCoefficients[i] = isSlowMode ? driveCoefficients[i] * SLOW_DRIVE_MULTIPLIER : driveCoefficients[i] * NORMAL_DRIVE_MULTIPLIER;
        }
        mecanum.driveRobotCentric(-driveCoefficients[0], -driveCoefficients[1], -driveCoefficients[2]);

        // --- ARM ---

        arm.read();
        arm.moveToSetPoint();

        if (ARM_STATE_FORWARD.wasJustPressed()) {
            if (arm.getState() == ArmComponent.State.PLACE_HIGH_BACKDROP) {
                arm.setState(ArmComponent.State.PLACE_LOW_BACKDROP);
            } else {
                arm.setState(ArmComponent.State.PLACE_HIGH_BACKDROP);
            }
        }

        if (ARM_STATE_DOWN.wasJustPressed()) {
            arm.setState(ArmComponent.State.PICKUP_GROUND);
        }

        // --- ACTIVE INTAKE  ---
        if (TURN_INTAKE_FORWARDS.wasJustPressed()) {
            if (!activeIntake.isTurning()) {
                activeIntake.turnForwards();
            } else {
                activeIntake.turnMotorOff();
            }
        }
        if (TURN_INTAKE_BACKWARDS.wasJustPressed()) {
            if (!activeIntake.isTurning()) {
                activeIntake.turnBackwards();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        // --- OUTTAKE ---


        if (OUTTAKE_TOGGLE_ALL_PIXEL.wasJustPressed()) {
            if (outtake.getBackServoPosition() == 0 && outtake.getFrontServoPosition() == 0) {
                outtake.releaseAll();
            } else {
                outtake.holdAll();
            }
        }

        if (OUTTAKE_TOGGLE_BACK_PIXEL.wasJustPressed()) {
            outtake.toggleBackOuttake();
        }

        // --- DRONE LAUNCHER ---

        if (DRONE_LEFT_RELEASE.wasJustPressed()) {
            droneServo.turnToAngle(180);
        }

        telemetry.addData("Drivetrain", isSlowMode ? "Slow Speed" : "Normal Speed");
        telemetry.addData("FRONT RELEASE", outtake.getFrontServoPosition() == 0 ? "HOLD" : "RELEASE");
        telemetry.addData("BACK RELEASE", outtake.getBackServoPosition() == 0 ? "HOLD" : "RELEASE");
        telemetry.addLine();
        telemetry.addData("Arm state", arm.getState());
        telemetry.addData("Arm position", arm.getArmMotor().getCurrentPosition());
        telemetry.addLine(activeIntake.isTurning() ? "Active Intake Turning" : "Active Intake not Turning");
        telemetry.update();
    }
}
