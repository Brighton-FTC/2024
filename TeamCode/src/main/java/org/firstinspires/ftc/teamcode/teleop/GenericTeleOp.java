package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSButtons;

public class GenericTeleOp extends OpMode {

    // P1 Controls

//    public static final GamepadKeys.Button OPERATION_VALOUR_BUTTON = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button OPERATION_VALOUR_BUTTON = PSButtons.CROSS;

    // P2 Controls

    public static final GamepadKeys.Button TOGGLE_LIFT_BUTTON = PSButtons.CROSS;
    public static final GamepadKeys.Button TOGGLE_GRABBER_BUTTON = PSButtons.CIRCLE;


    private GamepadEx playerGamepad1;
    private GamepadEx playerGamepad2;

    private ServoEx grabberServo;
    private GrabberComponent grabber;

    private ArmComponent arm;
    private MotorEx armMotor;
    private ServoEx grabberRotatorServo;

    @Override
    public void init() {
        playerGamepad1 = new GamepadEx(gamepad1);
        playerGamepad2 = new GamepadEx(gamepad2);

        grabberServo = new SimpleServo(hardwareMap, "grabber_servo",
                GrabberComponent.GRABBER_CLOSED_POSITION,
                GrabberComponent.GRABBER_OPEN_POSITION);
        grabber = new GrabberComponent(grabberServo);

        armMotor = new MotorEx(hardwareMap, "arm_motor");
        grabberRotatorServo = new SimpleServo(hardwareMap, "grabber_rotator_servo");
        arm = new ArmComponent(armMotor);

        playerGamepad2.getGamepadButton(TOGGLE_GRABBER_BUTTON).whenPressed(GrabberComponent::toggle);
    }

    @Override
    public void loop() {

    }
}
