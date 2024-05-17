package org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingMotorEx;
import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib.FTCLibCachingSimpleServo;

/**
 * Example of how to use the FTCLib wrappers.
 */
public class testWrappers extends OpMode {

    FTCLibCachingMotorEx testMotor;
    FTCLibCachingSimpleServo testServo;

    @Override
    public void init() {
        testMotor = new FTCLibCachingMotorEx(hardwareMap, "motor");
        testServo = new FTCLibCachingSimpleServo(hardwareMap, "servo", 0, 360);
        testServo.setChangeThreshold(0.05);
    }

    @Override
    public void loop() {
        testMotor.setHard(0.6);

        telemetry.addData("Change threshold of Motor", testMotor.getChangeThreshold());
    }
}
