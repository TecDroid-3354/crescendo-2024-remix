package net.tecdroid.config;

import com.revrobotics.CANSparkBase;

public class SwerveDriveConfig {
    private double[] azimuthalPidfCoefficients  = { 0.0, 0.0, 0.0, 0.0 };
    private double[] propulsionPidfCoefficients = { 0.0, 0.0, 0.0, 0.0 };

    public double[] getAzimuthalPidfCoefficients() {
        return azimuthalPidfCoefficients;
    }

    public double[] getPropulsionPidfCoefficients() {
        return propulsionPidfCoefficients;
    }
}
