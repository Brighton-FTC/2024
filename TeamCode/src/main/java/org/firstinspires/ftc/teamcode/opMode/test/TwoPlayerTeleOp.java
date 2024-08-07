package org.firstinspires.ftc.teamcode.opMode.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.teleop.PlayerButton;
import org.firstinspires.ftc.teamcode.util.inputs.PSButtons;

@TeleOp(name = "2 Driver TeleOp", group = "teleop-test")
public class TwoPlayerTeleOp extends GenericTeleOp {

    /**
     *     Change to use PlayerButtons as you want. Use a NullPlayerButton if you want that method to be disabled.
     */
    @Override
    public void init(){
        super.init();
        setButtons(
                new PlayerButton(gamepadp1, GamepadKeys.Button.RIGHT_BUMPER), // DRIVETRAIN_SLOW_MODE
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_RIGHT), // ARM_STATE_FORWARD
                new PlayerButton(gamepadp2, GamepadKeys.Button.DPAD_DOWN), // ARM_STATE_DOWN
                new PlayerButton(gamepadp2, PSButtons.TRIANGLE), // TURN_INTAKE_FORWARDS
                new PlayerButton(gamepadp2, PSButtons.SQUARE), // TURN_INTAKE_BACKWARDS
                new PlayerButton(gamepadp2, PSButtons.CIRCLE), // OUTTAKE_TOGGLE_ALL_PIXEL
                new PlayerButton(gamepadp2, PSButtons.CROSS), // OUTTAKE_TOGGLE_BACK_PIXEL
                new PlayerButton(gamepadp2, GamepadKeys.Button.LEFT_BUMPER) // DRONE_LEFT_RELEASE
        );
    }
}
