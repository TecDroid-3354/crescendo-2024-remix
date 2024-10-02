package net.tecdroid.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public final class SwerveDriveConstants {
    /**
     * Defines the index for accessing the front right module in a swerve module array
     */
    public static final int FRONT_RIGHT_MODULE_INDEX = 0;

    /**
     * Defines the index for accessing the front left module in a swerve module array
     */
    public static final int FRONT_LEFT_MODULE_INDEX  = 1;

    /**
     * Defines the index for accessing the back left module in a swerve module array
     */
    public static final int BACK_LEFT_MODULE_INDEX   = 2;

    /**
     * Defines the index for accessing the back right module in a swerve module array
     */
    public static final int BACK_RIGHT_MODULE_INDEX  = 3;

    /**
     * Defines the id of the front right swerve module in a swerve drive
     */
    public static final int FRONT_RIGHT_MODULE_ID = FRONT_RIGHT_MODULE_INDEX + 1;

    /**
     * Defines the id of the front left swerve module in a swerve drive
     */
    public static final int FRONT_LEFT_MODULE_ID  = FRONT_LEFT_MODULE_INDEX  + 1;

    /**
     * Defines the id of the back left swerve module in a swerve drive
     */
    public static final int BACK_LEFT_MODULE_ID   = BACK_LEFT_MODULE_INDEX   + 1;

    /**
     * Defines the id of the back right swerve module in a swerve drive
     */
    public static final int BACK_RIGHT_MODULE_ID  = BACK_RIGHT_MODULE_INDEX  + 1;

    /**
     * Describes the absolute value of the x-axis offset of a swerve module
     */
    public static final Measure<Distance> ABSOLUTE_MODULE_X_CENTER_OFFSET = Units.Meters.of(0.0);

    /**
     * Describes the absolute value of the y-axis offset of a swerve module
     */
    public static final Measure<Distance> ABSOLUTE_MODULE_Y_CENTER_OFFSET = Units.Meters.of(0.0);

    /**
     * Describes the center offset of the front right module in a swerve drive, in meters
     */
    public static final Translation2d FRONT_RIGHT_OFFSET_FROM_CENTER = new Translation2d(-ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), +ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));

    /**
     * Describes the center offset of the front left module in a swerve drive, in meters
     */
    public static final Translation2d FRONT_LEFT_OFFSET_FROM_CENTER  = new Translation2d(+ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), +ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));

    /**
     * Describes the center offset of the back left module in a swerve drive, in meters
     */
    public static final Translation2d BACK_LEFT_OFFSET_FROM_CENTER   = new Translation2d(+ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), -ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));

    /**
     * Describes the center offset of the back right module in a swerve drive, in meters
     */
    public static final Translation2d BACK_RIGHT_OFFSET_FROM_CENTER  = new Translation2d(-ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), -ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));
}
