package org.firstinspires.ftc.teamcode.util.gyro;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BCGyro extends RevIMU {
    public BCGyro(HardwareMap hw, String imuName) {
        super(hw, imuName);
    }

    public BCGyro(HardwareMap hw) {
        super(hw);
    }

    @Override
    public double getAbsoluteHeading() {
        // Return yaw
        return (720 - super.getAbsoluteHeading()) % 360;
    }
}
