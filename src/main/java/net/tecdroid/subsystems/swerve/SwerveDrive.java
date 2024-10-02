package net.tecdroid.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import net.tecdroid.config.SwerveDriveConfig;
import net.tecdroid.config.SwerveModuleConfig;
import net.tecdroid.constants.SwerveDriveConstants;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public final class SwerveDrive extends SubsystemBase {

    /**
     * Identifies the mode under which the swerve drive should operate
     */
    enum Mode {
        ROBOT__RELATIVE,
        FIELD_RELATIVE
    }

    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyroscope;

    @Getter
    private Mode mode;

    @Setter
    private Supplier<Measure<Velocity<Distance>>> xVelocitySource;

    @Setter
    private Supplier<Measure<Velocity<Distance>>> yVelocitySource;

    @Setter
    private Supplier<Measure<Velocity<Angle>>> aVelocitySource;

    public SwerveDrive() {
        modules[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX] = new SwerveModule(SwerveDriveConstants.FRONT_RIGHT_MODULE_ID, SwerveDriveConstants.FRONT_RIGHT_OFFSET_FROM_CENTER);
        modules[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX] = new SwerveModule(SwerveDriveConstants.FRONT_LEFT_MODULE_ID, SwerveDriveConstants.FRONT_LEFT_OFFSET_FROM_CENTER);
        modules[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX] = new SwerveModule(SwerveDriveConstants.BACK_LEFT_MODULE_ID, SwerveDriveConstants.BACK_LEFT_OFFSET_FROM_CENTER);
        modules[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX] = new SwerveModule(SwerveDriveConstants.BACK_RIGHT_MODULE_ID, SwerveDriveConstants.BACK_RIGHT_OFFSET_FROM_CENTER);

        kinematics = new SwerveDriveKinematics(
            modules[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX].getOffsetFromCenter(),
            modules[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX].getOffsetFromCenter(),
            modules[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX].getOffsetFromCenter(),
            modules[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX].getOffsetFromCenter()
        );

        gyroscope = new AHRS(I2C.Port.kMXP);

        this.xVelocitySource = () -> Units.MetersPerSecond.of(0.0);
        this.yVelocitySource = () -> Units.MetersPerSecond.of(0.0);
        this.aVelocitySource = () -> Units.RadiansPerSecond.of(0.0);

        setDefaultCommand(generateDefaultCommand());
    }

    /**
     * Loads the given configuration onto the module's components
     * @param driveConfig The configuration of the swerve drive this module belongs to
     * @param moduleConfig The configuration of this individual module
     */
    public void applyConfiguration(SwerveDriveConfig driveConfig, SwerveModuleConfig[] moduleConfig) {
        if (moduleConfig.length != 4) {
            throw new IllegalArgumentException("SwerveDrive#applyConfiguration requires four (4) module configuration objects");
        }

        modules[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX].applyConfiguration(driveConfig, moduleConfig[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX]);
        modules[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX].applyConfiguration(driveConfig, moduleConfig[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX]);
        modules[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX].applyConfiguration(driveConfig, moduleConfig[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX]);
        modules[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX].applyConfiguration(driveConfig, moduleConfig[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX]);
    }

    /**
     * Generates the default command for this subsystem
     * @return The default command
     */
    @NotNull
    public Command generateDefaultCommand() {
        return Commands.run(() -> {
            drive(getTargetXVelocity(), getTargetYVelocity(), getTargetAVelocity());
        });
    }

    /**
     * Causes the robot to move with the desired x-y velocities and angular velocity
     * @param xVelocity The x-velocity (Front-Positive)
     * @param yVelocity The y-velocity (Left-Positive)
     * @param aVelocity The angular velocity (CCW-Positive)
     */
    public void drive(Measure<Velocity<Distance>> xVelocity, Measure<Velocity<Distance>> yVelocity, Measure<Velocity<Angle>> aVelocity) {

        ChassisSpeeds speeds = null;

        switch (mode) {
            case ROBOT__RELATIVE -> speeds = new ChassisSpeeds(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond), aVelocity.in(Units.RadiansPerSecond));
            case FIELD_RELATIVE -> speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond), aVelocity.in(Units.RadiansPerSecond), getHeading());
        }

        SwerveModuleState[] states =  kinematics.toSwerveModuleStates(speeds);

        modules[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX].setOptimizedTargetState(states[SwerveDriveConstants.FRONT_RIGHT_MODULE_INDEX]);
        modules[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX].setOptimizedTargetState(states[SwerveDriveConstants.FRONT_LEFT_MODULE_INDEX]);
        modules[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX].setOptimizedTargetState(states[SwerveDriveConstants.BACK_LEFT_MODULE_INDEX]);
        modules[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX].setOptimizedTargetState(states[SwerveDriveConstants.BACK_RIGHT_MODULE_INDEX]);
    }

    /**
     * Retrieves the target x-velocity at which the drivetrain should move
     * @return The target velocity
     */
    public Measure<Velocity<Distance>> getTargetXVelocity() {
        return xVelocitySource.get();
    }

    /**
     * Retrieves the target y-velocity at which the drivetrain should move
     * @return The target velocity
     */
    public Measure<Velocity<Distance>> getTargetYVelocity() {
        return yVelocitySource.get();
    }

    /**
     * Retrieves the target angular velocity at which the drivetrain should rotate
     * @return The target velocity
     */
    public Measure<Velocity<Angle>> getTargetAVelocity() {
        return aVelocitySource.get();
    }

    /**
     * Retrieve the heading of the robot
     * @return The heading
     */
    public Rotation2d getHeading() {
        return new Rotation2d(gyroscope.getYaw());
    }

}
