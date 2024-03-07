package org.firstinspires.ftc.teamcode.teleop.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class PlayerButton {
    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;


    /**
     * Creates a gamepad button for triggering commands.
     */
    public PlayerButton(GamepadEx gamepad, @NonNull GamepadKeys.Button button) {
        this.gamepad = gamepad;
        this.button = button;
    }

    public void whenPressed(final Runnable runnable){
        gamepad.getGamepadButton(button).whenPressed(runnable);
    }

    public boolean isButtonPressed(){
        return gamepad.getButton(button);
    }

    public boolean wasJustPressed(){
        return gamepad.wasJustPressed(button);
    }
}