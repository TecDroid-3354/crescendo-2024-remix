package net.tecdroid.config;

public final class SwerveDriveConfig {
    /**
     * Defines the coefficients for the azimuthal motor's PIDF controller
     */
    private double[] azimuthalPidfCoefficients  = { 0.0, 0.0, 0.0, 0.0 };

    /**
     * Defines the coefficients for the propulsion motor's PIDF controller
     */
    private double[] propulsionPidfCoefficients = { 0.0, 0.0, 0.0, 0.0 };

    /**
     * Retrieves the azimuthal motor's PIDF coefficients for this config profile
     * @return The coefficient array
     */
    public double[] getAzimuthalPidfCoefficients() {
        return azimuthalPidfCoefficients;
    }

    /**
     * Retrieves the propulsion motor's PIDF coefficients for this config profile
     * @return The coefficient array
     */
    public double[] getPropulsionPidfCoefficients() {
        return propulsionPidfCoefficients;
    }
}
