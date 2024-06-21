package org.firstinspires.ftc.teamcode.util.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * A example of the null object pattern for PlayerButton - a dummy object that will do nothing.
 * It will only return the defaultBool passed to it in its constructor.
 *
 * If you want to disable a feature in the TeleOp for testing, just replace the button binded to the function with this.
 */
public class NullPlayerButton extends PlayerButton{

    private final boolean defaultBool;

    /**
     * @param gamepad the unused gamepad (set to either player, it's not used)
     * @param defaultBool the default value this object passes back in its return methods
     */
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