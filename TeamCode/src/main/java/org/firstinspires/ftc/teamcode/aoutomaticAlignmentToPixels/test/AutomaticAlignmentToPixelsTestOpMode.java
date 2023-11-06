package org.firstinspires.ftc.teamcode.aoutomaticAlignmentToPixels.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * An opmode showing how the automatic alignment to pixels should be implemented. <br />
 *
 * Controls:
 * <ul>
 *     <li>A (Cross on PlayStation) - Start/stop the automatic alignment to pixels</li>
 * </ul>
 */
@Disabled
@TeleOp(name = "Automatic Alignment To Pixels Test OpMode", group = "operation-valour-test")
public class AutomaticAlignmentToPixelsTestOpMode extends OpMode {
    private GamepadEx gamepad = new GamepadEx(gamepad1);

    private int selectedPixelStack = 0;

    @Override
    public void init() {
        AutomaticAlignmentToPixels.init(hardwareMap);

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            if (AutomaticAlignmentToPixels.isMoving()) {
                AutomaticAlignmentToPixels.startMoving();
            } else {
                AutomaticAlignmentToPixels.stopMoving();
            }
        });
    }

    @Override
    public void loop() {
        AutomaticAlignmentToPixels.goToPixelStack(selectedPixelStack);
    }

    @Override
    public void stop() {
        AutomaticAlignmentToPixels.stop();
    }
}
