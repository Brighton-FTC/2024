package org.firstinspires.ftc.teamcode.teleop.util;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.teleop.GenericTeleOp;

public class NullPlayerButton extends PlayerButton{

    private final boolean defaultBool;
    public NullPlayerButton(boolean defaultBool) {
        super(PlayerCount.P1, GamepadKeys.Button.A);
        this.defaultBool = defaultBool;
    }

    @Override
    public void whenPressed(final Runnable runnable){}

    @Override
    public boolean isButtonPressed(){
        return defaultBool;
    }
}