package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.util.PlayerButton;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "2 Driver TeleOp", group = "teleop-test")
public class TwoPlayerTeleOp extends GenericTeleOp {

    /**
     *     Change to use PlayerButtons as you want. Use a NullPlayerButton if you want that method to be disabled.
     */
    public TwoPlayerTeleOp() {
        super();
        setButtons(
                new PlayerButton(gamepadp1, GamepadKeys.Button.RIGHT_BUMPER), // DRIVETRAIN_SLOW_MODE
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_RIGHT), // ARM_STATE_FORWARD
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_LEFT), // ARM_STATE_BACKWARDS
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_DOWN), // ARM_STATE_DOWN
                new PlayerButton(gamepadp2, PSButtons.CIRCLE), // TURN_INTAKE_CONSTANT
                new PlayerButton(gamepadp2, PSButtons.SQUARE), // TURN_INTAKE_MANUAL
                new PlayerButton(gamepadp1, PSButtons.TRIANGLE), // OUTTAKE_RELEASE_ALL_PIXEL
                new PlayerButton(gamepadp1, PSButtons.CROSS), // OUTTAKE_RELEASE_ONE_PIXEL
                new PlayerButton(gamepadp2, GamepadKeys.Button.LEFT_BUMPER) // DRONE_LEFT_RELEASE
        );
    }

    @Override
    public void start() {
        gamepad1.rumbleBlips(Integer.MAX_VALUE);
        gamepad1.runLedEffect(new Gamepad.LedEffect.Builder()
                .addStep(255, 0, 0, 100)
                .addStep(0, 0, 0, 100)
                .setRepeating(true)
                .build());

        super.start();
    }
}
