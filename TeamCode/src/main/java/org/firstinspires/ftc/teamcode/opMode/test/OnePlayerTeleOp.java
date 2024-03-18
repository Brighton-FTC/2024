package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.util.inputs.PlayerButton;

@TeleOp(name = "1 Driver TeleOp", group = "teleop")
public class OnePlayerTeleOp extends GenericTeleOp {

    /**
     * Change to use PlayerButtons as you want. Use {@link PlayerButton#NULL_KEYBIND} if you want that method to be disabled.
     */
    public OnePlayerTeleOp() {
        super();

        setButtons(
                PlayerButton.NULL_KEYBIND, // slow mode
                PlayerButton.NULL_KEYBIND, // enable/disable drive train

                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_RIGHT), // cycle arm state forwards
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_LEFT), // cycle arm state backwards
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_UP), // move arm to selected state
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_DOWN), // move arm to ground

                new PlayerButton(gamepadp1, PSButtons.CROSS), // toggle intake
                new PlayerButton(gamepadp1, PSButtons.CIRCLE), // turn intake once

                new PlayerButton(gamepadp1, PSButtons.TRIANGLE), // release all pixels
                new PlayerButton(gamepadp1, PSButtons.CROSS), // release one pixel

                new PlayerButton(gamepadp1, GamepadKeys.Button.RIGHT_BUMPER) // release drone
        );
    }
}
