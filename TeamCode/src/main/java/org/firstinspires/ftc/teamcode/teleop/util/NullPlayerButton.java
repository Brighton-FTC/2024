package org.firstinspires.ftc.teamcode.teleop.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class NullPlayerButton extends PlayerButton{

    private final boolean defaultBool;
    public NullPlayerButton(GamepadEx gamepad, boolean defaultBool) {
        super(gamepad, GamepadKeys.Button.A);
        this.defaultBool = defaultBool;
    }

    @Override
    public void whenPressed(final Runnable runnable){}

    @Override
    public boolean isButtonPressed(){
        return defaultBool;
    }

    @Override
    public boolean wasJustPressed(){
        return defaultBool;
    }
}