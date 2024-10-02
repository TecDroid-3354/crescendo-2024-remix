package net.tecdroid.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.tecdroid.config.SwerveDriveConfig;
import net.tecdroid.config.SwerveModuleConfig;
import net.tecdroid.constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
    private static final int AZIMUTHAL_MOTOR_ID_OFFSET = 0;
    private static final int PROPULSION_MOTOR_ID_OFFSET = 1;
    private static final int AZIMUTHAL_ENCODER_ID_OFFSET = 2;

    private SwerveModuleState targetState;

    private final Translation2d offsetFromCenter;

    private final CANSparkMax azimuthalMotorController;
    private final RelativeEncoder azimuthalMotorEncoder;
    private final SparkPIDController azimuthalPidfController;

    private final CANSparkMax propulsionMotorController;
    private final RelativeEncoder propulsionMotorEncoder;
    private final SparkPIDController propulsionPidfController;

    private final CANcoder azimuthalReferenceEncoder;

    public SwerveModule(final int moduleNumber, Translation2d offsetFromCenter) {
        this.targetState = new SwerveModuleState(0.0, new Rotation2d(0.0));
        this.offsetFromCenter = offsetFromCenter;

        this.azimuthalMotorController = new CANSparkMax((moduleNumber * 10) + AZIMUTHAL_MOTOR_ID_OFFSET, CANSparkLowLevel.MotorType.kBrushless);
        this.azimuthalMotorEncoder = azimuthalMotorController.getEncoder();
        this.azimuthalPidfController = azimuthalMotorController.getPIDController();

        this.azimuthalMotorController.clearFaults();
        this.azimuthalMotorController.setIdleMode(SwerveModuleConstants.AZIMUTHAL_MOTOR_IDLE_MODE);
        this.azimuthalMotorController.setOpenLoopRampRate(SwerveModuleConstants.AZIMUTHAL_RAMP_RATE.in(Units.Seconds));
        this.azimuthalMotorController.setClosedLoopRampRate(SwerveModuleConstants.AZIMUTHAL_RAMP_RATE.in(Units.Seconds));
        this.azimuthalMotorController.setSmartCurrentLimit((int)SwerveModuleConstants.AZIMUTHAL_MOTOR_MAX_CURRENT_DRAW.in(Units.Amps));
        this.azimuthalMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.AZIMUTHAL_ENCODER_POSITION_CONVERSION_FACTOR.in(Units.Radians));
        this.azimuthalMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.AZIMUTHAL_ENCODER_VELOCITY_CONVERSION_FACTOR.in(Units.RadiansPerSecond));
        this.azimuthalPidfController.setPositionPIDWrappingEnabled(true);
        this.azimuthalPidfController.setPositionPIDWrappingMinInput(SwerveModuleConstants.AZIMUTHAL_ENCODER_INPUT_MIN.in(Units.Radians));
        this.azimuthalPidfController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.AZIMUTHAL_ENCODER_INPUT_MAX.in(Units.Radians));
        this.azimuthalPidfController.setOutputRange(SwerveModuleConstants.MIN_PID_OUTPUT, SwerveModuleConstants.MAX_PID_OUTPUT);

        this.propulsionMotorController = new CANSparkMax((moduleNumber * 10) + PROPULSION_MOTOR_ID_OFFSET, CANSparkLowLevel.MotorType.kBrushless);
        this.propulsionMotorEncoder = propulsionMotorController.getEncoder();
        this.propulsionPidfController = propulsionMotorController.getPIDController();

        this.propulsionMotorController.clearFaults();
        this.propulsionMotorController.setIdleMode(SwerveModuleConstants.PROPULSION_MOTOR_IDLE_MODE);
        this.propulsionMotorController.setOpenLoopRampRate(SwerveModuleConstants.PROPULSION_RAMP_RATE.in(Units.Seconds));
        this.propulsionMotorController.setClosedLoopRampRate(SwerveModuleConstants.PROPULSION_RAMP_RATE.in(Units.Seconds));
        this.propulsionMotorController.setSmartCurrentLimit((int)SwerveModuleConstants.PROPULSION_MOTOR_MAX_CURRENT_DRAW.in(Units.Amps));
        this.propulsionMotorEncoder.setPositionConversionFactor(SwerveModuleConstants.PROPULSION_ENCODER_POSITION_CONVERSION_FACTOR.in(Units.Meters));
        this.propulsionMotorEncoder.setVelocityConversionFactor(SwerveModuleConstants.PROPULSION_ENCODER_VELOCITY_CONVERSION_FACTOR.in(Units.MetersPerSecond));
        this.propulsionPidfController.setOutputRange(SwerveModuleConstants.MIN_PID_OUTPUT, SwerveModuleConstants.MAX_PID_OUTPUT);

        this.azimuthalReferenceEncoder = new CANcoder((moduleNumber * 10) + AZIMUTHAL_ENCODER_ID_OFFSET);

        MagnetSensorConfigs azimuthalMagnetSensorConfigs = new MagnetSensorConfigs();
        azimuthalMagnetSensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        azimuthalMagnetSensorConfigs.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        CANcoderConfigurator azimuthalEncoderConfigurator = this.azimuthalReferenceEncoder.getConfigurator();
        azimuthalEncoderConfigurator.clearStickyFaults();
        azimuthalEncoderConfigurator.apply(azimuthalMagnetSensorConfigs);

        this.azimuthalReferenceEncoder.optimizeBusUtilization();

        setDefaultCommand(generateDefaultCommand());
    }

    public void applyConfiguration(final SwerveDriveConfig driveConfig, final SwerveModuleConfig moduleConfig) {
        double[] azimuthalPidfCoefficients = driveConfig.getAzimuthalPidfCoefficients();
        azimuthalMotorController.setInverted(moduleConfig.isAzimuthalInverted());
        azimuthalPidfController.setP(azimuthalPidfCoefficients[0]);
        azimuthalPidfController.setI(azimuthalPidfCoefficients[1]);
        azimuthalPidfController.setD(azimuthalPidfCoefficients[2]);
        azimuthalPidfController.setFF(azimuthalPidfCoefficients[3]);

        double[] propulsionPidfCoefficients = driveConfig.getAzimuthalPidfCoefficients();
        propulsionMotorController.setInverted(moduleConfig.isPropulsionInverted());
        propulsionPidfController.setP(propulsionPidfCoefficients[0]);
        propulsionPidfController.setI(propulsionPidfCoefficients[1]);
        propulsionPidfController.setD(propulsionPidfCoefficients[2]);
        propulsionPidfController.setFF(propulsionPidfCoefficients[3]);
    }

    public Command generateDefaultCommand() {
        return Commands.run(() -> {
            final SwerveModuleState target = getTargetState();
            azimuthalPidfController.setReference(target.angle.getRadians(), CANSparkBase.ControlType.kPosition);
            propulsionPidfController.setReference(target.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        });
    }

    public void adjustToReference() {
        final double referenceAngleDegrees = azimuthalReferenceEncoder.getAbsolutePosition().getValue();
        azimuthalMotorEncoder.setPosition(referenceAngleDegrees);
    }

    public void zeroReferenceEncoder() {
        azimuthalReferenceEncoder.setPosition(0.0);
    }

    public void resetPosition() {
        propulsionMotorEncoder.setPosition(0.0);
    }

    public SwerveModuleState optimizeState(SwerveModuleState state) {
        return SwerveModuleState.optimize(state, getAzimuth());
    }

    public void setOptimizedTargetState(SwerveModuleState state) {
        setTargetState(optimizeState(state));
    }

    public Rotation2d getAzimuth() {
        return Rotation2d.fromDegrees(azimuthalMotorEncoder.getPosition());
    }

    public Measure<Distance> getPositionMeters() {
        return Units.Meters.of(propulsionMotorEncoder.getPosition());
    }

    public Measure<Velocity<Distance>> getVelocityMetersPerSecond() {
        return Units.MetersPerSecond.of(propulsionMotorEncoder.getVelocity());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters().in(Units.Meters), getAzimuth());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
    }

    public Translation2d getOffsetFromCenter() {
        return offsetFromCenter;
    }
}
