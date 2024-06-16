package org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.cachingftclib;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.cachinghardwaredevice.CachingServo;

/**
 * This is the same thing as {@link com.arcrobotics.ftclib.hardware.SimpleServo}, but using CachingServo instead of Servo.
 * Use this over SimpleServo. See wiki page for why caching motor writes is useful.
 */
public class FTCLibCachingSimpleServo implements ServoEx {
    private final CachingServo servo;

    //always stored internally as radians
    private double maxAngle, minAngle;

    private final double maxPosition = 1;
    private final double minPosition = 0;

    public FTCLibCachingSimpleServo(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        servo = new CachingServo(hw.get(Servo.class, servoName));

        this.minAngle = toRadians(minAngle, angleUnit);
        this.maxAngle = toRadians(maxAngle, angleUnit);
    }

    public FTCLibCachingSimpleServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        this(hw, servoName, minDegree, maxDegree, AngleUnit.DEGREES);
    }

    public void setChangeThreshold(double changeThreshold) {
        this.servo.setChangeThreshold(changeThreshold);
    }

    public double getChangeThreshold() {
        return this.servo.getChangeThreshold();
    }

    @Override
    public void rotateByAngle(double angle, AngleUnit angleUnit) {
        angle = getAngle(angleUnit) + angle;
        turnToAngle(angle, angleUnit);
    }

    @Override
    public void rotateByAngle(double degrees) {
        rotateByAngle(degrees, AngleUnit.DEGREES);
    }

    @Override
    public void turnToAngle(double angle, AngleUnit angleUnit) {
        double angleRadians = Range.clip(toRadians(angle, angleUnit), minAngle, maxAngle);
        setPosition((angleRadians - minAngle) / (getAngleRange(AngleUnit.RADIANS)));
    }

    @Override
    public void turnToAngle(double degrees) {
        turnToAngle(degrees, AngleUnit.DEGREES);
    }

    @Override
    public void rotateBy(double position) {
        position = getPosition() + position;
        setPosition(position);
    }

    @Override
    public void setPosition(double position) {
        servo.setPosition(Range.clip(position, minPosition, maxPosition));
    }

    @Override
    public void setRange(double min, double max, AngleUnit angleUnit) {
        this.minAngle = toRadians(min, angleUnit);
        this.maxAngle = toRadians(max, angleUnit);
    }

    @Override
    public void setRange(double min, double max) {
        setRange(min, max, AngleUnit.DEGREES);
    }

    @Override
    public void setInverted(boolean isInverted) {
        servo.setDirection(isInverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    @Override
    public boolean getInverted() {
        return Servo.Direction.REVERSE == servo.getDirection();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public double getAngle(AngleUnit angleUnit) {
        return getPosition() * getAngleRange(angleUnit) + fromRadians(minAngle, angleUnit);
    }

    @Override
    public double getAngle() {
        return getAngle(AngleUnit.DEGREES);
    }

    public double getAngleRange(AngleUnit angleUnit) {
        return fromRadians(maxAngle - minAngle, angleUnit);
    }

    public double getAngleRange() {
        return getAngleRange(AngleUnit.DEGREES);
    }

    @Override
    public void disable() {
        servo.close();
    }

    @Override
    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "SimpleServo: " + port + "; " + controller;
    }

    private double toRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toRadians(angle) : angle;
    }

    private double fromRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(angle) : angle;
    }
}
