package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.inputs.PSButtons;

@TeleOp(name = "TeleOp", group = "teleop-test")
public class TwoPlayerTeleOp extends GenericTeleOp {

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
}
