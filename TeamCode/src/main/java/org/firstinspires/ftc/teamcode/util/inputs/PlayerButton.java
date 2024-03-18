package org.firstinspires.ftc.teamcode.util.inputs;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Button class that contains which gamepad it is binded to.
 * Exposes some methods of GamepadEx and GamepadKeys.Button.
 * If you want to use a method from those classes that's not here, just add it (and also override it in NullPlayerButton).
 */
public class PlayerButton {
    public static final PlayerButton NULL_KEYBIND = new NullPlayerButton();

    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;


    /**
     * Creates a gamepad button for triggering commands.
     */
    public PlayerButton(GamepadEx gamepad, @NonNull GamepadKeys.Button button) {
        this.gamepad = gamepad;
        this.button = button;
    }

    public void whenPressed(final Runnable runnable) {
        gamepad.getGamepadButton(button).whenPressed(runnable);
    }

    public boolean isButtonPressed() {
        return gamepad.getButton(button);
    }

    public boolean wasJustPressed() {
        return gamepad.wasJustPressed(button);
    }

    /**
     * A example of the null object pattern for PlayerButton - a dummy object that will do nothing.
     * It will only return the defaultBool passed to it in its constructor.
     * <p>
     * If you want to disable a feature in the TeleOp for testing, just replace the button binded to the function with this.
     */
    private static class NullPlayerButton extends PlayerButton {

        public NullPlayerButton() {
            super(null, GamepadKeys.Button.A);
        }

        @Override
        public void whenPressed(final Runnable runnable) {
        }

        @Override
        public boolean isButtonPressed() {
            return false;
        }

        @Override
        public boolean wasJustPressed() {
            return false;
        }
    }
}