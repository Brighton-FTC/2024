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

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    // alternate between 0 and 2000
    public static int target = 0;

    private final double ticks_in_degrees = 560.0 / 360.0;

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
        int slidePos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos ", slidePos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}