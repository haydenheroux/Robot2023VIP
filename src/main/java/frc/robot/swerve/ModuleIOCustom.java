package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;
import frc.robot.swerve.SteerMotorIO.SteerMotorValues;

/**
 * Controls a swerve module.
 *
 * <p>Each swerve module has a motor that controls the angle of the wheel, a motor that drives the
 * wheel, and an encoder that tracks the position of the wheel. When the module is initialized, the
 * position of the angle motor will be set to the position of the wheel, and the position of the
 * drive motor will be set to zero.
 */
public class ModuleIOCustom implements ModuleIO {

  public final ModuleConfiguration config;

  private final SteerMotorIO steerMotor;
  private final SteerMotorValues steerMotorValues = new SteerMotorValues();

  private final DriveMotorIO driveMotor;
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

  private final AzimuthEncoderIO azimuthEncoder;
  private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

  private SwerveModuleState setpoint = new SwerveModuleState();

  /**
   * Constructs a new module using the provided configuration.
   *
   * @param config the configuration to use to construct the module.
   */
  public ModuleIOCustom(ModuleConfiguration config) {
    this.config = config;

    steerMotor = Swerve.FACTORY.createSteerMotor(config);
    driveMotor = Swerve.FACTORY.createDriveMotor(config);
    azimuthEncoder = Swerve.FACTORY.createAzimuthEncoder(config);

    steerMotor.configure();
    driveMotor.configure();

    azimuthEncoder.configure();

    azimuthEncoder.updateValues(azimuthEncoderValues);

    steerMotor.setPosition(azimuthEncoderValues.angleRotations);
  }

  // TODO call periodically
  /** Updates values with sensor information. */
  private void update() {
    azimuthEncoder.updateValues(azimuthEncoderValues);

    steerMotor.updateValues(steerMotorValues);
    driveMotor.updateValues(driveMotorValues);
  }

  public void setSetpoint(SwerveModuleState setpoint) {
    final SwerveModuleState optimizedSetpoint = optimizeSetpoint(setpoint);

    this.setpoint = optimizedSetpoint;

    steerMotor.setSetpoint(optimizedSetpoint.angle.getRotations());
    driveMotor.setVelocitySetpoint(optimizedSetpoint.speedMetersPerSecond);
  }

  private SwerveModuleState optimizeSetpoint(SwerveModuleState setpoint) {
    setpoint =
        SwerveModuleState.optimize(
            setpoint, Rotation2d.fromRotations(steerMotorValues.angleRotations));

    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/bf960378bca7fe3f32c46d3d529925d960d1ff37/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/Module.java#L117
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(getState().angle).getCos();

    return setpoint;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }

  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }
}
