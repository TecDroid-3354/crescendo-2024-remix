package net.tecdroid.config;

import lombok.Getter;

public class SwerveModuleConfig {
    private boolean azimuthalInverted  = false;
    private boolean propulsionInverted = false;

    public boolean isAzimuthalInverted() {
        return azimuthalInverted;
    }

    public boolean isPropulsionInverted() {
        return propulsionInverted;
    }
}
