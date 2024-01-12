package org.firstinspires.ftc.teamcode.teleop.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.teleop.GenericTeleOp;

public class PlayerButton {
    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;

    private static GamepadEx gamepadp1;
    private static GamepadEx gamepadp2;

    /**
     * Creates a gamepad button for triggering commands.
     */
    public PlayerButton(PlayerCount players, @NonNull GamepadKeys.Button button) {
        if (players == PlayerCount.P1){
            this.gamepad = gamepadp1;
        }
        else {
            this.gamepad = gamepadp2;
        }
        this.button = button;
    }

    public void whenPressed(final Runnable runnable){
        gamepad.getGamepadButton(button).whenPressed(runnable);
    }

    public boolean isButtonPressed(){
        return gamepad.getButton(button);
    }

    public void setGamepads(GamepadEx gamepad1, GamepadEx gamepad2){
        gamepadp1 = gamepad1;
        gamepadp2 = gamepad2;
    }
}