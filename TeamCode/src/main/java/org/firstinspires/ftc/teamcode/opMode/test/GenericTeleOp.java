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

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.HeadingPID;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.util.inputs.PlayerButton;

import java.util.Arrays;

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
    private final double MANUAL_TURN_THRESHOLD = 0.1;
    private final double DEAD_ZONE_SIZE = 0.2;

    private boolean isSlowMode = false;
    private boolean isDrivetrainEnabled = true;

    private ArmComponent.State selectedState;

    private PlayerButton drivetrainSlowMode;
    private PlayerButton drivetrainEnabled;
    private PlayerButton armStateForward;
    private PlayerButton armStateBackwards;
    private PlayerButton armUp;
    private PlayerButton armDown;
    private PlayerButton toggleIntake;
    private PlayerButton turnIntakeManual;
    private PlayerButton outtakeReleaseAllPixel;
    private PlayerButton outtakeReleaseOnePixel;
    private PlayerButton droneLeftRelease;

    private HeadingPID headingPID;

    private IMU gyro;

    private ElapsedTime time;
    private double previousTurning;

    protected void setButtons(
            PlayerButton drivetrainSlowMode,
            PlayerButton drivetrainEnabled,
            PlayerButton armStateForward,
            PlayerButton armStateBackwards,
            PlayerButton armUp,
            PlayerButton armDown,
            PlayerButton toggleIntake,
            PlayerButton turnIntakeManual,
            PlayerButton outtakeReleaseAllPixel,
            PlayerButton outtakeReleaseOnePixel,
            PlayerButton droneLeftRelease
    ) {
        this.drivetrainSlowMode = drivetrainSlowMode;
        this.drivetrainEnabled = drivetrainEnabled;
        this.armStateForward = armStateForward;
        this.armStateBackwards = armStateBackwards;
        this.armUp = armUp;
        this.armDown = armDown;
        this.toggleIntake = toggleIntake;
        this.turnIntakeManual = turnIntakeManual;
        this.outtakeReleaseAllPixel = outtakeReleaseAllPixel;
        this.outtakeReleaseOnePixel = outtakeReleaseOnePixel;
        this.droneLeftRelease = droneLeftRelease;
    }

    @Override
    public void init() {
        gamepadp1 = new GamepadEx(gamepad1);

        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        headingPID = new HeadingPID(telemetry, gyro);

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"),
                new SimpleServo(hardwareMap, "outtake_rotation_servo", 0, 360),
                lynxModule.getInputVoltage(VoltageUnit.VOLTS));
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

        time.startTime();
    }

    @Override
    public void loop() {
        gamepadp1.readButtons();
        gamepadp2.readButtons();

        // drivetrain
        if (drivetrainSlowMode.wasJustPressed()) {
            isSlowMode = !isSlowMode;
        }

        if (drivetrainEnabled.wasJustPressed()) {
            isDrivetrainEnabled = !isDrivetrainEnabled;
        }

        if (isDrivetrainEnabled) {
            double timePassed = time.seconds();
            double headingCorrection = headingPID.runPID(timePassed, previousTurning);
            double[] driveCoefficients = {
                    gamepadp1.getLeftX(),
                    gamepadp1.getLeftY(),
                    Math.abs(gamepadp1.getRightX()) > MANUAL_TURN_THRESHOLD ? gamepadp1.getRightX() : headingCorrection
            };

            for (int i = 0; i < driveCoefficients.length; i++) {
                driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
                driveCoefficients[i] *= isSlowMode ? SLOW_DRIVE_MULTIPLIER : NORMAL_DRIVE_MULTIPLIER;
                driveCoefficients[i] = Range.clip(driveCoefficients[i], -1, 1); // clip in case of multiplier that is greater than 1
            }
            mecanum.driveRobotCentric(driveCoefficients[0], driveCoefficients[1], driveCoefficients[2]);
            previousTurning = driveCoefficients[2];

            telemetry.addData("Forward speed", driveCoefficients[1]);
            telemetry.addData("Strafe speed", driveCoefficients[0]);
            telemetry.addData("Turn speed", driveCoefficients[2]);

            if (Math.abs(gamepadp1.getRightX()) > MANUAL_TURN_THRESHOLD) {
                telemetry.addLine("\tTurning manually");
            }

        } else {
            mecanum.driveRobotCentric(0, 0, 0);
            previousTurning = 0;

            telemetry.addLine("Drivetrain frozen");
        }

        telemetry.addLine();

        // components
        arm.read();
        arm.moveToSetPoint();

        if (arm.getState() == ArmComponent.State.PICKUP_GROUND && arm.atSetPoint()) {
            activeIntake.moveMotor();
        }

        if (armStateForward.wasJustPressed()) {
            int newIndex = Arrays.asList(ArmComponent.State.values()).indexOf(selectedState) + 1;
            if (newIndex > ArmComponent.State.values().length) {
                newIndex = 2; // ignore the first two states
            }
            selectedState = ArmComponent.State.values()[newIndex];
        }

        if (armStateBackwards.wasJustPressed()) {
            int newIndex = Arrays.asList(ArmComponent.State.values()).indexOf(arm.getState()) - 1;
            if (newIndex < 2) { // ignore the first two states
                newIndex = ArmComponent.State.values().length;
            }
            selectedState = ArmComponent.State.values()[newIndex];
        }

        if (armUp.wasJustPressed()) {
            arm.setState(selectedState);
        }

        if (armDown.wasJustPressed()) {
            arm.setState(ArmComponent.State.PLACE_GROUND);
        }

        if (toggleIntake.wasJustPressed()) {
            if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                arm.setState(ArmComponent.State.PLACE_GROUND);
                activeIntake.turnContinually();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        if (turnIntakeManual.wasJustPressed()) {
            activeIntake.turnManually();
        }

        if (outtakeReleaseOnePixel.wasJustPressed()) {
            outtake.releasePixel();
        }

        if (outtakeReleaseAllPixel.wasJustPressed()) {
            outtake.releaseAllPixels();
        }

        if (droneLeftRelease.wasJustPressed()) {
            droneLauncher.launch();
        }

        time.reset();

        telemetry.addData("Arm state", arm.getState().toString());
        telemetry.addData("Selected arm state", selectedState.toString());
        telemetry.addData("Active intake state", activeIntake.getState().toString());
        telemetry.addLine(outtake.isClosed() ? "Outtake closed" : "Outtake open");
        telemetry.addLine(droneLauncher.getDroneLaunched() ? "Drone launched" : "Drone not launched");
        telemetry.update();
    }
}
