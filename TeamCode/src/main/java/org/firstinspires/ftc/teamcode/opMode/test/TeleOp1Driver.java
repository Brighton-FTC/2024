package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.test.ActiveIntakeComponent;
import org.firstinspires.ftc.teamcode.components.test.ArmComponent;
import org.firstinspires.ftc.teamcode.components.test.DroneLauncherComponent;
import org.firstinspires.ftc.teamcode.components.test.OuttakeComponent;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

/**
 * 1 driver teleop.
 * <hr />
 * Controls:
 * <ul>
 *     <li>Left joystick y: move robot forwards/backwards</li>
 *     <li>Left joystick x: move robot left/right</li>
 *     <li>Right joystick x: turn robot</li>
 *
 *     <li>Left bumper: toggle slow mode</li>
 *
 *     <li>Dpad left/right: change selected arm state. </li>
 *     <li>Dpad up: move arm to selected arm state</li>
 *     <li>Dpad down: move arm to ground</li>
 *
 *     <li>Cross: turn active intake to take in one pixel</li>
 *     <li>Square: toggle active intake (turning continuously/off)</li>
 *
 *     <li>Triangle: release one pixel</li>
 *     <li>Circle: release all pixels</li>
 *
 *     <li>Right bumper: launch drone</li>
 *     <li>Dpad left: turn active intake continuously (and lower arm if arm is not lowered)</li>
 *     <li>Dpad right: turn active intake to take one pixel</li>
 * </ul>
 */
@TeleOp(name = "2 Driver TeleOp", group = "teleop-test")
public class TeleOp1Driver extends OpMode {
    private GamepadEx gamepad;

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

    @Override
    public void init() {
        gamepad = new GamepadEx(gamepad1);

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
        gamepad.readButtons();

        // drivetrain
        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            isSlowMode = !isSlowMode;
        }

        double[] driveCoefficients = {gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX()};
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

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if (selectedState == ArmComponent.State.LOW) {
                selectedState = ArmComponent.State.MIDDLE;
            } else if (selectedState == ArmComponent.State.MIDDLE) {
                selectedState = ArmComponent.State.HIGH;
            } else {
                selectedState = ArmComponent.State.LOW;
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (selectedState == ArmComponent.State.LOW) {
                selectedState = ArmComponent.State.HIGH;
            } else if (selectedState == ArmComponent.State.MIDDLE) {
                selectedState = ArmComponent.State.LOW;
            } else {
                selectedState = ArmComponent.State.MIDDLE;
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            arm.setState(selectedState);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            arm.setState(ArmComponent.State.GROUND);
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                arm.setState(ArmComponent.State.GROUND);
                activeIntake.turnContinually();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        if (gamepad.wasJustPressed(PSButtons.CROSS)) {
            activeIntake.turnManually();
        }

        if (gamepad.wasJustPressed(PSButtons.SQUARE)) {
            if (activeIntake.getState() == ActiveIntakeComponent.State.OFF) {
                activeIntake.turnContinually();
            } else {
                activeIntake.turnMotorOff();
            }
        }

        if (gamepad.wasJustPressed(PSButtons.TRIANGLE)) {
            outtake.releasePixel();
        }

        if (gamepad.wasJustPressed(PSButtons.CIRCLE)) {
            outtake.releaseAllPixels();
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            droneLauncher.launch();
        }

        telemetry.addData("Arm state", arm.getState());
        telemetry.addData("Selected arm state", selectedState);
        telemetry.addData("Active intake state", activeIntake.getState());
        telemetry.addLine(outtake.isClosed() ? "Outtake closed" : "Outtake open");
        telemetry.addLine(droneLauncher.getDroneLaunched() ? "Drone launched" : "Drone not launched");
    }
}
