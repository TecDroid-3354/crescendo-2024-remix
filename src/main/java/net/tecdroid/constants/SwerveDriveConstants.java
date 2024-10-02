package net.tecdroid.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class SwerveDriveConstants {
    public static final int FRONT_RIGHT_MODULE_INDEX = 0;
    public static final int FRONT_LEFT_MODULE_INDEX  = 1;
    public static final int BACK_LEFT_MODULE_INDEX   = 2;
    public static final int BACK_RIGHT_MODULE_INDEX  = 3;

    public static final int FRONT_RIGHT_MODULE_ID = FRONT_RIGHT_MODULE_INDEX + 1;
    public static final int FRONT_LEFT_MODULE_ID  = FRONT_LEFT_MODULE_INDEX  + 1;
    public static final int BACK_LEFT_MODULE_ID   = BACK_LEFT_MODULE_INDEX   + 1;
    public static final int BACK_RIGHT_MODULE_ID  = BACK_RIGHT_MODULE_INDEX  + 1;

    public static final Measure<Distance> ABSOLUTE_MODULE_X_CENTER_OFFSET = Units.Meters.of(0.0);
    public static final Measure<Distance> ABSOLUTE_MODULE_Y_CENTER_OFFSET = Units.Meters.of(0.0);

    public static final Translation2d FRONT_RIGHT_OFFSET_FROM_CENTER = new Translation2d(-ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), +ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));
    public static final Translation2d FRONT_LEFT_OFFSET_FROM_CENTER  = new Translation2d(+ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), +ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));
    public static final Translation2d BACK_LEFT_OFFSET_FROM_CENTER   = new Translation2d(+ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), -ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));
    public static final Translation2d BACK_RIGHT_OFFSET_FROM_CENTER  = new Translation2d(-ABSOLUTE_MODULE_X_CENTER_OFFSET.in(Units.Meter), -ABSOLUTE_MODULE_Y_CENTER_OFFSET.in(Units.Meter));
}
