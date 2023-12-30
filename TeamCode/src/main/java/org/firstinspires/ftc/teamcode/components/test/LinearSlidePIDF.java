package org.firstinspires.ftc.teamcode.components.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Class for tuning the PIDF Controller in {@link LinearSlideComponent}.
 * Must be used with FTC Dashboard.
 * Modify the coefficients on the dashboard and alternate between -100 and -1000 as the target.
 * Changes to fields are only applied when you hit the save button.
 */
@Config
@TeleOp( name = "Linear Slide PIDF", group = "linear-slide-test")
public class LinearSlidePIDF extends OpMode {
    public static int POSITION_ERROR = 5;

    private PIDController controller;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static int target = 0;
    private DcMotorEx slide_motor;

    public static boolean atsetpoint = false;

    private ArmComponent arm;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        controller.setTolerance(3);

        arm = new ArmComponent(new MotorEx(hardwareMap, "arm_motor"));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide_motor = hardwareMap.get(DcMotorEx.class, "linear_slide_motor");
    }

    @Override
    public void loop() {
        int slidePos = slide_motor.getCurrentPosition();

        controller.setPID(p, i, d);
        double pid = controller.calculate(slidePos, target);
        double ff = Math.sin(Math.toRadians(arm.getArmPosition() / arm.arm_ticks_in_degrees)) * f;

        double power = pid + ff;

        slide_motor.setPower(power);

        atsetpoint = false;

        if (controller.atSetPoint()){
            slide_motor.setPower(0.05);
            atsetpoint = true;
        }
        telemetry.addData("at setpoint?: ", atsetpoint);
        telemetry.addData("pos ", slidePos);
        telemetry.addData("target ", target);
        telemetry.update();
        arm.read();
    }
}
