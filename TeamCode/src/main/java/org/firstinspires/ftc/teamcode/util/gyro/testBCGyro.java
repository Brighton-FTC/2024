package org.firstinspires.ftc.teamcode.util.gyro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Temporary BC Gyro Test", group = "gyro_test")
public class testBCGyro extends OpMode {

    public static BCGyro gyro;

    @Override
    public void init() {
        gyro = new BCGyro(hardwareMap);
        gyro.init();
    }

    @Override
    public void loop() {
        telemetry.addData("Gyro absolute heading: ", gyro.getAbsoluteHeading());
        telemetry.addData("Gyro relative heading: ", gyro.getHeading());
        telemetry.update();
    }
}
