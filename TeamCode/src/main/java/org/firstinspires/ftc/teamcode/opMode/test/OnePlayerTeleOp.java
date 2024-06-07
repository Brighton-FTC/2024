package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.teleop.PlayerButton;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "1 Driver TeleOp", group = "teleop-test")
public class OnePlayerTeleOp extends GenericTeleOp {

    /**
     *     Change to use PlayerButtons as you want. Use a NullPlayerButton if you want that method to be disabled.
     */
    public OnePlayerTeleOp() {
        super();
        setButtons(
                new PlayerButton(gamepadp1, GamepadKeys.Button.RIGHT_BUMPER), // DRIVETRAIN_SLOW_MODE
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_RIGHT), // ARM_STATE_FORWARD
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_LEFT), // ARM_STATE_BACKWARDS
                new PlayerButton(gamepadp1, GamepadKeys.Button.DPAD_DOWN), // ARM_STATE_DOWN
                new PlayerButton(gamepadp1, PSButtons.CIRCLE), // TURN_INTAKE_CONSTANT
                new PlayerButton(gamepadp1, PSButtons.TRIANGLE), // OUTTAKE_RELEASE_ALL_PIXEL
                new PlayerButton(gamepadp1, PSButtons.CROSS), // OUTTAKE_RELEASE_ONE_PIXEL
                new PlayerButton(gamepadp1, GamepadKeys.Button.LEFT_BUMPER) // DRONE_LEFT_RELEASE
        );
    }
}
