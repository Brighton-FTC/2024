package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Noe Mecanum Drive Test", group = "test")
public class MecanumDriveTest extends OpMode {
    public static final double DEAD_ZONE_THRESHOLD = 0.2;
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "back_left_drive"),
                new Motor(hardwareMap, "back_right_drive")
        );
    }

    @Override
    public void loop() {
        double[] speeds = {gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x};
        for (int i = 0; i < speeds.length; i++){
            if (Math.abs(speeds[i]) < DEAD_ZONE_THRESHOLD){
                speeds[i] = 0;
            }
        }



        drive.driveRobotCentric(speeds[1], speeds[0], speeds[2]);

        telemetry.addData("forward speed", speeds[0]);
        telemetry.addData("strafe speed", speeds[1]);
        telemetry.addData("turn speed", speeds[2]);
    }
}
