package org.firstinspires.ftc.teamcode.components.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Code to lift/lower arm. Also moves grabber when arm is lifted or lowered. <br />
 * Controls:
 * <ul>
 *     <li>Lift/Lower Arm: B (Circle on PlayStation controller)</li>
 * </ul>
 */


@Disabled
@TeleOp(name = "Arm Component", group = "components_test")
public class ArmComponent extends OpMode {
    // TODO: fill in these values
    public final int GRABBER_TILT_DOWN_POSITION = 0;
    public final int GRABBER_TILT_UP_POSITION = 180;
    public final int ARM_LIFTED_POSITION = 0;
    public final int ARM_NONLIFTED_POSITION = 2000;

    // TODO: A value should be assigned to this when the class is turned into a component instead of a OpMode
    // Keeping this as a OpMode so it's a little easier to test, will convert later
    private final int initialArmPositionCounts = 200;

    // TODO: Tune somehow to make sure is correct
    // Speed which makes robot 'just' start to move - static friction
    private final int STATIC_FRICTION_CONST = 170;

    public static final GamepadKeys.Button SET_ARM_LIFTED = GamepadKeys.Button.B;

    private ServoEx grabberTiltServo;

    private boolean isArmLifted = false;

    private final GamepadEx gamepad = new GamepadEx(gamepad1);

    // TODO: Tune this
    private final PIDFController pidf = new PIDFController(0,0,0, 0);
    private Motor armMotor;

    @Override
    public void init() {
        // TODO: fill in device names
        grabberTiltServo = new SimpleServo(hardwareMap, "servo_name", 0, 360);
        armMotor = new Motor(hardwareMap, "motorOne");

        // set ranges on servos, just in case
        grabberTiltServo.setRange(GRABBER_TILT_DOWN_POSITION, GRABBER_TILT_UP_POSITION);

        armMotor.setRunMode(Motor.RunMode.RawPower);

        gamepad.getGamepadButton(SET_ARM_LIFTED).whenPressed(new InstantCommand(() -> {
            isArmLifted = !isArmLifted;
            if (isArmLifted){
                setTargetPosition(ARM_LIFTED_POSITION);
            }
            else {
                setTargetPosition(ARM_NONLIFTED_POSITION);
            }
        }));

    }

    @Override
    public void start() {
        // set servo and motor to their starting positions
        grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
        setTargetPosition(ARM_NONLIFTED_POSITION);
    }

    /**
     * Set PID and feedforward to desired position
     * @param position The desired final position of the arm
     */
    private void setTargetPosition(int position) {
        pidf.setSetPoint(position - initialArmPositionCounts);
    }

    @Override
    public void loop() {
        if (isArmLifted) {
            grabberTiltServo.setPosition(GRABBER_TILT_UP_POSITION);
        } else {
            grabberTiltServo.setPosition(GRABBER_TILT_DOWN_POSITION);
        }

        double velocity = 0;
        velocity += pidf.calculate(armMotor.getCurrentPosition());
        velocity += STATIC_FRICTION_CONST;
        armMotor.set(velocity);

        telemetry.addData("Grabber Tilt Angle", grabberTiltServo.getAngle());
        telemetry.addData("Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Motor Speed", armMotor.encoder.getRawVelocity());
        telemetry.addData("Motor Acceleration", armMotor.encoder.getAcceleration());
    }
}