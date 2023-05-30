package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Robot;
import frc.robot.swerve.AngleMotorIO.AngleMotorIOValues;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;

/**
 * Controls a swerve module.
 *
 * <p>Each swerve module has a motor that controls the angle of the wheel, a motor that drives the
 * wheel, and an encoder that tracks the position of the wheel. When the module is initialized, the
 * position of the angle motor will be set to the position of the wheel, and the position of the
 * drive motor will be set to zero.
 */
public class Module implements TelemetryOutputter {

  public final ModuleConfiguration config;

  private final AngleMotorIO angleMotor;
  private final AngleMotorIOValues angleMotorValues = new AngleMotorIOValues();

  private final DriveMotorIO driveMotor;
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

  private final AzimuthEncoderIO azimuthEncoder;
  private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

  /**
   * Constructs a new module using the provided configuration.
   *
   * @param config the configuration to use to construct the module.
   */
  public Module(ModuleConfiguration config) {
    this.config = config;

    if (Robot.isSimulation()) {
      angleMotor = new AngleMotorIOSim();
      driveMotor = new DriveMotorIOSim();
      azimuthEncoder = new AzimuthEncoderIOSim();
    } else {
      angleMotor = new AngleMotorIOTalonFX(config.can.angle, config.can.bus);
      driveMotor = new DriveMotorIOTalonFX(config.can.drive, config.can.bus);
      azimuthEncoder =
          new AzimuthEncoderIOCANcoder(
              config.can.azimuth, config.can.bus, config.azimuthOffsetRotations);
    }

    angleMotor.configure();
    driveMotor.configure();

    azimuthEncoder.configure();

    azimuthEncoder.updateValues(azimuthEncoderValues);

    angleMotor.setPosition(azimuthEncoderValues.angleRotations);
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    ShuffleboardLayout layout = tab.getLayout(this.config.name, BuiltInLayouts.kList);
    layout.addNumber(
        "Absolute Angle (deg)",
        () -> Units.rotationsToDegrees(azimuthEncoderValues.angleRotations));
    layout.addNumber("Angle (deg)", () -> getState().angle.getDegrees());
    layout.addNumber(
        "Omega (dps)", () -> Units.rotationsToDegrees(angleMotorValues.omegaRotationsPerSecond));
    layout.addNumber("Velocity (mps)", () -> getState().speedMetersPerSecond);
  }

  @Override
  public void outputTelemetry() {}

  /** Updates values with sensor information. */
  public void update() {
    azimuthEncoder.updateValues(azimuthEncoderValues);

    angleMotor.updateValues(angleMotorValues);
    driveMotor.updateValues(driveMotorValues);
  }

  /**
   * Sets the module setpoint.
   *
   * @param setpoint
   * @param force true if module should skip optimization.
   */
  public void setSetpoint(SwerveModuleState setpoint, boolean force) {
    setpoint =
        SwerveModuleState.optimize(
            setpoint, Rotation2d.fromRotations(angleMotorValues.angleRotations));

    driveMotor.setVelocitySetpoint(setpoint.speedMetersPerSecond);

    if (force == false) {
      SwerveMath.dejitter(setpoint, Rotation2d.fromRotations(angleMotorValues.angleRotations));
    }

    // TODO Check for wheel rotating greater than one rotation
    double angleSetpoint = setpoint.angle.getRotations();
    double closestAngleSetpoint =
        SwerveMath.placeInClosest0To1Scope(angleSetpoint, angleMotorValues.angleRotations);

    angleMotor.setSetpoint(closestAngleSetpoint);
  }

  /**
   * Gets the state of the module.
   *
   * @return the state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }

  /**
   * Gets the module position relative to the initial position.
   *
   * @return the position relative to the initial position.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }
}
