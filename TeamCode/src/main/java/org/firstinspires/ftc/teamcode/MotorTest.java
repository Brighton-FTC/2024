package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Motor Test", group = "test")
public class MotorTest extends OpMode {
    public static final String DATA_PATH = "/data_points.txt";

    private double speed = -1;

    private List<String> lines;

    private MotorEx motor;


    @Override
    public void init() {
        motor = new MotorEx(hardwareMap, "motor");

        lines = new ArrayList<>();
    }

    @Override
    public void loop() {
        motor.set(1);

        telemetry.addData("Speed (-1 to 1)", speed);
        telemetry.addData("Speed (encoder ticks per second)", motor.getVelocity());

        lines.add(String.format("%f, %f", speed, motor.getVelocity()));

        speed += 0.05;
    }

    @Override
    public void stop() {
        try {
            PrintWriter writer = new PrintWriter(DATA_PATH);

            for (String line : lines) {
                writer.println(line);
            }

            writer.close();

        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}