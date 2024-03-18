package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.util.inputs.PlayerButton;

@TeleOp(name = "2 Driver TeleOp", group = "teleop")
public class TwoPlayerTeleOp extends GenericTeleOp {

    /**
     * Change to use PlayerButtons as you want. Use {@link PlayerButton#NULL_KEYBIND} if you want that method to be disabled.
     */
    public TwoPlayerTeleOp() {
        super();

        setButtons(
                new PlayerButton(gamepadp1, GamepadKeys.Button.RIGHT_BUMPER), // slow mode
                new PlayerButton(gamepadp1, GamepadKeys.Button.LEFT_BUMPER), // enable/disable drivetrain

                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_RIGHT), // cycle arm state forwards
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_LEFT), // cycle arm state backwards
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_UP), // move arm to selected state
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_DOWN), // move arm to ground

                new PlayerButton(gamepadp2, PSButtons.CROSS), // toggle intake
                new PlayerButton(gamepadp2, PSButtons.CIRCLE), // turn intake off

                new PlayerButton(gamepadp2, PSButtons.TRIANGLE), // release all pixels
                new PlayerButton(gamepadp2, PSButtons.CROSS), // release one pixel

                new PlayerButton(gamepadp2, GamepadKeys.Button.RIGHT_BUMPER) // release drone
        );
    }

    @Override
    public void start() {
        // prank for lawrence, remove for comp
        gamepad1.rumbleBlips(Integer.MAX_VALUE);
        gamepad1.runLedEffect(new Gamepad.LedEffect.Builder()
                .addStep(255, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .setRepeating(true)
                .build());

        super.start();
    }
}
