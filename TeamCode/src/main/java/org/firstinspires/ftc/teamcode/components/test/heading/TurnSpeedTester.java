package org.firstinspires.ftc.teamcode.components.test.heading;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.gyro.BCGyro;

/**
 * This class corrects for our drivetrain curving.
 * Essentially, our (mecanum) drivetrain has minor problems driving straight and major problems strafing
 * This is due to the 90 degree gearboxes being only on the front motors. (poverty yay!)
 *
 * To fix this, we use Heading PID to get a value to add to the turnSpeed.
 * The setpoint is what heading the drivetrain SHOULD be at (based on turn inputs from gamepad).
 * This is saved in desiredHeading.
 * Then, the actual heading of the bot (based on gyro) is passed into the controller, and a value is returned!
 */
@Autonomous(name = "Heading PID Tuner", group = "tuner")
public class TurnSpeedTester extends OpMode {

    private BCGyro gyro;

    private MecanumDrive drive;

    private ElapsedTime time;

    @Override
    public void init() {
        this.gyro = new BCGyro(hardwareMap);
        gyro.init();

        drive = new MecanumDrive(new MotorEx(hardwareMap, "front_left_drive"),
                                 new MotorEx(hardwareMap, "front_right_drive"),
                                 new MotorEx(hardwareMap, "back_left_drive"),
                                 new MotorEx(hardwareMap, "back_right_drive"));
        time = new ElapsedTime();
        time.startTime();
    }

    @Override
    public void loop() {
        if (time.milliseconds() < 5000) {
            drive.driveRobotCentric(0, 1, 0);
        }
        telemetry.addData("Gyro: ", gyro.getHeading());
        telemetry.update();
    }
}
