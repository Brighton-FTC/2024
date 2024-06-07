package org.firstinspires.ftc.teamcode.components.test.heading;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
@Autonomous(name = "Heading PID Tester", group = "tuner")
public class HeadingPIDTester extends OpMode {

    private BCGyro gyro;

    private MecanumDrive drive;

    private ElapsedTime time;

    private HeadingPID headingPID;

    private GamepadEx gamepad;

    private final double DEAD_ZONE_SIZE = 0.2;

    private double previousTurning;

    @Override
    public void init() {
        this.gyro = new BCGyro(hardwareMap);
        gyro.init();

        drive = new MecanumDrive(new MotorEx(hardwareMap, "front_left_drive"),
                                 new MotorEx(hardwareMap, "front_right_drive"),
                                 new MotorEx(hardwareMap, "back_left_drive"),
                                 new MotorEx(hardwareMap, "back_right_drive"));
        time = new ElapsedTime();
        time.reset();

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        double timePassed = time.milliseconds();
        double headingCorrection = headingPID.runPID(timePassed, previousTurning);
        double[] driveCoefficients = {
                gamepad.getLeftX(),
                gamepad.getLeftY(),
                gamepad.getRightX() + headingCorrection
        };
        for (int i = 0; i < driveCoefficients.length; i++) {
            driveCoefficients[i] = Math.abs(driveCoefficients[i]) > DEAD_ZONE_SIZE ? driveCoefficients[i] : 0;
            driveCoefficients[i] = Range.clip(-1, 1, driveCoefficients[i]); // clip in case of multiplier that is greater than 1
        }
        drive.driveRobotCentric(driveCoefficients[0], driveCoefficients[1], driveCoefficients[2]);
        previousTurning = gamepad.getLeftY();
        time.reset();

        telemetry.addData("Previous turn input: ", previousTurning);
        telemetry.addData("Time passed: ", timePassed);
        telemetry.addData("Heading correction: ", headingCorrection);
        telemetry.update();
    }
}
