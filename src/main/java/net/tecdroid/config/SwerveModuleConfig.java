package net.tecdroid.config;

public final class SwerveModuleConfig {

    /**
     * Defines whether the azimuthal motor is inverted
     */
    private boolean azimuthalInverted  = false;

    /**
     * Defines whether the propulsion motor is inverted
     */
    private boolean propulsionInverted = false;

    /**
     * Retrieves whether the azimuthal motor is inverted for this config profile
     * @return The inversion state
     */
    public boolean isAzimuthalInverted() {
        return azimuthalInverted;
    }

    /**
     * Retrieves whether the propulsion motor is inverted for this config profile
     * @return The inversion state
     */
    public boolean isPropulsionInverted() {
        return propulsionInverted;
    }
}
