package org.firstinspires.ftc.teamcode.components.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Class for tuning the PIDF Controller in {@link ArmComponent}.
 * Must be used with FTC Dashboard.
 * Modify the coefficients on the dashboard and alternate between -100 and -1000 as the target.
 * Changes to fields are only applied when you hit the save button.
 */
@Config
@TeleOp( name = "Arm PIDF", group = "arm-test")
public class ArmPIDF extends OpMode {
    private PIDController controller;

    public static boolean isGreaterThan = false;

    public static int APEX_TICKS = 0;

    // at rest = 235 horzional = 10
    // 0.0005 d, 0.01 p, tested without container attached

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    // alternate between 0 and 2000
    public static int target = 0;

    // TODO: MAKE SURE THIS IS CORRECT.
    // learnroadrunner.com tells me HD REV has 560 ticks per rev, but according to Mo we use CORE REV.
    // CORE REV doesn't have much info on it - I found a google doc saying CORE is 288.
    // However, past me left this value as 560, so either my original source was wrong, CORE uses 560, or we aren't using CORE.
    private final double ticks_in_degrees = 288.0 / 360.0;

    private static final double INITIAL_RADS = -225;

    private DcMotorEx arm_motor;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPosition = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPosition, target);
        double ff = Math.cos(Math.toRadians(armPosition / ticks_in_degrees) - INITIAL_RADS) * f;

        double power;
//        if (isGreaterThan) {
//            power = arm_motor.getCurrentPosition() > APEX_TICKS ? pid + ff : pid - ff;
//        } else {
//            power = arm_motor.getCurrentPosition() < APEX_TICKS ? pid + ff : pid - ff;
//        }

        power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos ", armPosition);
        telemetry.addData("vel", arm_motor.getVelocity());
        telemetry.addData("target ", target);
        telemetry.addData("PID", pid);
        telemetry.addData("FF", ff);
        telemetry.addData("PIDF", power);
        telemetry.update();
    }
}
