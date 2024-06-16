package org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.CachingDcMotorEX;

/**
 * This is the same thing as {@link com.arcrobotics.ftclib.hardware.motors.MotorEx}, but using CachingDcMotorEX instead of DcMotorEX.
 * Use this over MotorEx. See wiki page for why caching motor writes is useful.
 */
public class FTCLibCachingMotorEx extends Motor {

    public CachingDcMotorEX motorEx;

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public FTCLibCachingMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public FTCLibCachingMotorEx(@NonNull HardwareMap hMap, String id, @NonNull Motor.GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        motorEx = (CachingDcMotorEX) super.motor;
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public FTCLibCachingMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (CachingDcMotorEX) super.motor;
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            motorEx.setPowerResult(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motorEx.setPowerResult(output * error);
        } else {
            motorEx.setPowerResult(output);
        }
    }

    /**
     * Ignores normal caching, write is let through unless cached power is exactly the same.
     * Use for PID deadzones and other use cases where a motor MUST be some power.
     *
     * @param output a value from 1.0 to -1.0 that represents power of the motor
     */
    public void setHard(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            motorEx.setPowerResultHard(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motorEx.setPowerResultHard(output * error);
        } else {
            motorEx.setPowerResultHard(output);
        }
    }

    /**
     * @param velocity the velocity in ticks per second
     */
    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    public void setChangeThreshold(double changeThreshold) {
        motorEx.setChangeThreshold(changeThreshold);
    }

    public double getChangeThreshold() {
        return motorEx.getChangeThreshold();
    }

    /**
     * Sets the velocity of the motor to an angular rate
     *
     * @param velocity  the angular rate
     * @param angleUnit radians or degrees
     */
    public void setVelocity(double velocity, AngleUnit angleUnit) {
        setVelocity(getCPR() * AngleUnit.RADIANS.fromUnit(angleUnit, velocity) / (2 * Math.PI));
    }

    /**
     * @return the velocity of the motor in ticks per second
     */
    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    /**
     * @return the acceleration of the motor in ticks per second squared
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }
}
