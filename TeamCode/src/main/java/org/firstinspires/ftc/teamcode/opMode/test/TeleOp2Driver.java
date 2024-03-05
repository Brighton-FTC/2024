package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

/**
 * 2 driver teleop.
 * <hr />
 * Controls:
 * <br .
 * PLayer 1:
 * <ul>
 *     <li>Left joystick y: move robot forwards/backwards</li>
 *     <li>Left joystick x: move robot left/right</li>
 *     <li>Right joystick x: turn robot</li>
 *     <li>Right/left bumper: toggle slow mode</li>
 * </ul>
 *
 * Player 2:
 * <ul>
 *     <li>Circle: toggle arm</li>
 *     <li>Cross: toggle linear slide (and lower arm if intake is active)</li>
 *     <li>Triangle: release pixel</li>
 *     <li>Square: release drone</li>
 * </ul>
 */
@TeleOp(name = "2 Driver TeleOp", group = "teleop-test")
public class TeleOp2Driver extends OpMode {
    private GamepadEx gamepadEx1, gamepadEx2;

    private ArmComponent arm;
    private DroneLauncherComponent droneLauncher;
    private ActiveIntakeComponent activeIntake;
    private OuttakeComponent outtake;

    private MecanumDrive mecanum;

    private final double NORMAL_DRIVE_MULTIPLIER = 1;
    private final double SLOW_DRIVE_MULTIPLIER = 0.25;
    private final double DEAD_ZONE_SIZE = 0.2;
    private boolean isSlowMode;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));
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
        // prank: don't remove until lawrence has experienced
        gamepad1.rumble(Integer.MAX_VALUE);
        gamepad1.runLedEffect(new Gamepad.LedEffect.Builder()
                .addStep(255, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .setRepeating(true)
                .build());
    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        // drivetrain
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                || gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            isSlowMode = !isSlowMode;
        }

        double[] driveCoefficients = {gamepadEx1.getLeftX(), gamepadEx1.getLeftY(), gamepadEx1.getRightX()};
        for (int i = 0; i < driveCoefficients.length; i++) {
            driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
            driveCoefficients[i] = isSlowMode ? driveCoefficients[i] * SLOW_DRIVE_MULTIPLIER : driveCoefficients[i] * NORMAL_DRIVE_MULTIPLIER;
            driveCoefficients[i] = Range.clip(-1, 1, driveCoefficients[i]); // clip in case of multiplier that is greater than 1
        }
        mecanum.driveRobotCentric(driveCoefficients[0], driveCoefficients[1], driveCoefficients[2]);

        // components
        arm.read();
        arm.moveToSetPoint();

        if (!arm.isLifted() && arm.atSetPoint()) {
            activeIntake.moveMotor();
        }

        if (gamepadEx2.wasJustPressed(PSButtons.CIRCLE)) {
            arm.toggle();
        }

        if (gamepadEx2.wasJustPressed(PSButtons.CROSS)) {
            if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                arm.lower();
                activeIntake.turnContinually();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        if (gamepadEx2.wasJustPressed(PSButtons.TRIANGLE)) {
            outtake.releasePixel();
        }

        if (gamepadEx2.wasJustPressed(PSButtons.SQUARE)) {
            droneLauncher.launch();
        }
    }
}
