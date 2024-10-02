package net.tecdroid.constants;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.units.*;

public final class SwerveModuleConstants {

    ///////////////////////////////////
    // Encoder Factors and Modifiers //
    ///////////////////////////////////

    /**
     * Describes the diameter of the wheels in a module
     */
    public static final Measure<Distance> WHEEL_DIAMETER = Units.Meters.of(0.1016);

    /**
     * Describes the circumference of the wheels in a module
     */
    public static final Measure<Distance> WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

    /**
     * Describes the gear ratio applied to the output of azimuthal motors
     */
    public static final double AZIMUTHAL_GEAR_RATIO = 150.0 / 7.0;

    /**
     * Describes the gear ratio applied to the output of propulsion motors
     */
    public static final double PROPULSION_GEAR_RATIO = 6.12;

    /**
     * Defines the value that the encoders of the azimuthal motors should report when a full turn occurs
     */
    public static final Measure<Angle> AZIMUTHAL_ENCODER_DESIRED_OUTPUT_PER_ROTATION = Units.Radians.of(2.0 * Math.PI);

    /**
     * Defines the value that the encoders of the propulsion motors should report when the wheel rotates once
     */
    public static final Measure<Distance> PROPULSION_ENCODER_DESIRED_OUTPUT_PER_ROTATION = WHEEL_CIRCUMFERENCE;

    /**
     * Describes the position conversion factor that should be applied to the output of steer encoder
     * This particular conversion converts to degrees
     */
    public static final Measure<Angle> AZIMUTHAL_ENCODER_POSITION_CONVERSION_FACTOR = AZIMUTHAL_ENCODER_DESIRED_OUTPUT_PER_ROTATION.divide(AZIMUTHAL_GEAR_RATIO);

    /**
     * Describes the position conversion factor that should be applied to the output of steer encoder
     * This particular conversion converts to deg/s
     */
    public static final Measure<Velocity<Angle>> AZIMUTHAL_ENCODER_VELOCITY_CONVERSION_FACTOR = AZIMUTHAL_ENCODER_POSITION_CONVERSION_FACTOR.per(Units.Seconds.of(60.0));

    /**
     * Describes the position conversion factor that should be applied to the output of drive encoder readings
     * This particular conversion converts to meters
     */
    public static final Measure<Distance> PROPULSION_ENCODER_POSITION_CONVERSION_FACTOR = PROPULSION_ENCODER_DESIRED_OUTPUT_PER_ROTATION.divide(PROPULSION_GEAR_RATIO);

    /**
     * Describes the position conversion factor that should be applied to the output of drive encoder readings
     * This particular conversion converts to m/s
     */
    public static final Measure<Velocity<Distance>> PROPULSION_ENCODER_VELOCITY_CONVERSION_FACTOR = PROPULSION_ENCODER_POSITION_CONVERSION_FACTOR.per(Units.Seconds.of(60.0));

    //////////////////////////
    // Motor Current Limits //
    //////////////////////////

    /**
     * Defines the max amperage that azimuthal motors may draw
     */
    public static final Measure<Current> AZIMUTHAL_MOTOR_MAX_CURRENT_DRAW = Units.Amps.of(30.0);

    /**
     * Defines the max amperage that propulsion motors may draw
     */
    public static final Measure<Current> PROPULSION_MOTOR_MAX_CURRENT_DRAW = Units.Amps.of(40.0);

    ///////////////////////
    // Motor Ins n' Outs //
    ///////////////////////

    /**
     * Defines the time it takes the azimuthal motor to reach its set speed
     */
    public static final Measure<Time> AZIMUTHAL_RAMP_RATE = Units.Seconds.of(0.0);

    /**
     * Defines the time it takes the propulsion motor to reach its set speed
     */
    public static final Measure<Time> PROPULSION_RAMP_RATE = Units.Second.of(0.0);

    /**
     * Defines the min output that a PIDF is able to feed a motor
     */
    public static final double MIN_PID_OUTPUT = -1.0;

    /**
     * Defines the max output that a PIDF is able to feed a motor
     */
    public static final double MAX_PID_OUTPUT = 1.0;

    //////////////////
    // Motor Status //
    //////////////////

    /**
     * Defines the idle mode of the azimuthal motor
     */
    public static final CANSparkBase.IdleMode AZIMUTHAL_MOTOR_IDLE_MODE = CANSparkBase.IdleMode.kBrake;

    /**
     * Defines the idle mode of the propulsion motor
     */
    public static final CANSparkBase.IdleMode PROPULSION_MOTOR_IDLE_MODE = CANSparkBase.IdleMode.kBrake;

    ////////////////////
    // Encoder Status //
    ////////////////////

    /**
     * Defines the minimum input to which the azimuthal encoder should wrap to
     */
    public static final Measure<Angle> AZIMUTHAL_ENCODER_INPUT_MIN = Units.Radians.of(-Math.PI);

    /**
     * Defines the maximum input to which the azimuthal encoder should wrap to
     */
    public static final Measure<Angle> AZIMUTHAL_ENCODER_INPUT_MAX = Units.Radians.of(Math.PI);

}
