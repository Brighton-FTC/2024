package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.teleop.util.PlayerButton;
import org.firstinspires.ftc.teamcode.teleop.util.PlayerCount;

@TeleOp(name = "TeleOp", group = "teleop-test")
public class TwoPlayerTeleOp extends GenericTeleOp {

    // P1 Controls
//    public static final GamepadKeys.Button DPAD_STRAFE_LEFT = GamepadKeys.Button.DPAD_LEFT;
//    public static final GamepadKeys.Button DPAD_STRAFE_RIGHT = GamepadKeys.Button.DPAD_RIGHT;
//    public static final GamepadKeys.Button DPAD_FORWARD = GamepadKeys.Button.DPAD_UP;
//    public static final GamepadKeys.Button DPAD_BACKWARDS = GamepadKeys.Button.DPAD_DOWN;
//
//
//    // P2 Controls
//    public static final GamepadKeys.Button TOGGLE_ARM_BUTTON = PSButtons.CROSS;
//    public static final GamepadKeys.Button TOGGLE_GRABBER_BUTTON = PSButtons.CIRCLE;
//    public static final GamepadKeys.Button TOGGLE_LINEAR_SLIDE_BUTTON = PSButtons.SQUARE;
//    public static final GamepadKeys.Button DRONE_LAUNCH_1_BUTTON = GamepadKeys.Button.LEFT_BUMPER;
//    public static final GamepadKeys.Button DRONE_LAUNCH_2_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;

    /**
     *     Functions, in order <ol>
     *     <li> DPAD_STRAFE_LEFT = GamepadKeys.Button.DPAD_LEFT;</li>
     *     <li> DPAD_STRAFE_RIGHT = GamepadKeys.Button.DPAD_RIGHT;</li>
     *     <li> DPAD_FORWARD = GamepadKeys.Button.DPAD_UP;</li>
     *     <li> DPAD_BACKWARDS = GamepadKeys.Button.DPAD_DOWN;</li>
     *     <li> TOGGLE_ARM_BUTTON = PSButtons.CROSS;</li>
     *     <li> TOGGLE_GRABBER_BUTTON = PSButtons.CIRCLE;</li>
     *     <li> TOGGLE_LINEAR_SLIDE_BUTTON = PSButtons.SQUARE;</li>
     *     <li> DRONE_LAUNCH_1_BUTTON = GamepadKeys.Button.LEFT_BUMPER;</li>
     *     <li> DRONE_LAUNCH_2_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;</li>
     *     </ol>
     *
     *     Change to use PlayerButtons as you want. Use a NullPlayerButton if you want that method to be disabled.
     */
    protected TwoPlayerTeleOp() {
        super(new PlayerButton(PlayerCount.P1, GamepadKeys.Button.DPAD_LEFT), // DPAD_STRAFE_LEFT
                new PlayerButton(PlayerCount.P1, GamepadKeys.Button.DPAD_RIGHT), // DPAD_STRAFE_RIGHT
                new PlayerButton(PlayerCount.P1, GamepadKeys.Button.DPAD_UP), // DPAD_FORWARD
                new PlayerButton(PlayerCount.P1, GamepadKeys.Button.DPAD_DOWN), // DPAD_BACKWARDS
                new PlayerButton(PlayerCount.P2, PSButtons.CROSS), // TOGGLE_ARM_BUTTON
                new PlayerButton(PlayerCount.P2, PSButtons.CIRCLE), // TOGGLE_GRABBER_BUTTON
                new PlayerButton(PlayerCount.P2, PSButtons.SQUARE), // TOGGLE_LINEAR_SLIDE_BUTTON
                new PlayerButton(PlayerCount.P2, GamepadKeys.Button.LEFT_BUMPER), // DRONE_LAUNCH_1_BUTTON
                new PlayerButton(PlayerCount.P2, GamepadKeys.Button.RIGHT_BUMPER)); // DRONE_LAUNCH_2_BUTTON
    }
}
