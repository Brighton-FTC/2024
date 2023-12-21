package org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.CachingCRServo;

/**
 * This is the same thing as {@link com.arcrobotics.ftclib.hardware.motors.CRServo}, but using CachingCRServo instead of the default CRServo.
 * CRServo is both a interface in the SDK and a class in FTCLib that implements the interface, which is confusing.
 * Use this over FTCLib's implementation of CRServo. See wiki page for why caching motor writes is useful.
 */
public class FTCLibCachingCRServo extends Motor {
    /**
     * The constructor for the CR Servo.
     */
    protected CachingCRServo crServo;

    /**
     * The constructor for the CR Servo.
     */
    public FTCLibCachingCRServo(HardwareMap hMap, String id) {
        crServo = new CachingCRServo(hMap.get(com.qualcomm.robotcore.hardware.CRServo.class, id));
    }

    @Override
    public void set(double output) {
        crServo.setPower(output);
    }

    @Override
    public double get() {
        return crServo.getPower();
    }

    public void setChangeThreshold(double changeThreshold) {
        crServo.setChangeThreshold(changeThreshold);
    }

    public double getChangeThreshold() {
        return crServo.getChangeThreshold();
    }

    @Override
    public void setInverted(boolean isInverted) {
        crServo.setDirection(isInverted ? com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE
                : com.qualcomm.robotcore.hardware.CRServo.Direction.FORWARD);
    }

    @Override
    public boolean getInverted() {
        return crServo.getDirection() == com.qualcomm.robotcore.hardware.CRServo.Direction.REVERSE;
    }

    @Override
    public void disable() {
        crServo.close();
    }

    public void stop() {
        set(0);
    }

    @Override
    public void stopMotor() {
        stop();
    }
}
