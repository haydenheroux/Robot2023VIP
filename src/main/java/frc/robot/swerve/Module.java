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
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
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
public class Module implements TelemetryOutputter {

  public final ModuleConfiguration config;

  private final SteerMotorIO steerMotor;
  private final SteerMotorValues steerMotorValues = new SteerMotorValues();
  private double steerMotorSetpointAngleRotations = 0.0;

  private final DriveMotorIO driveMotor;
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();
  private double driveMotorSetpointVelocityMetersPerSecond = 0.0;

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
      steerMotor = new SteerMotorIOSim();
      driveMotor = new DriveMotorIOSim();
      azimuthEncoder = new AzimuthEncoderIOSim();
    } else {
      steerMotor = new SteerMotorIOTalonFX(config.can.angle, config.can.azimuth, config.can.bus);
      driveMotor = new DriveMotorIOTalonFX(config.can.drive, config.can.bus);
      azimuthEncoder =
          new AzimuthEncoderIOCANcoder(
              config.can.azimuth, config.can.bus, config.azimuthOffsetRotations);
    }

    steerMotor.configure();
    driveMotor.configure();

    azimuthEncoder.configure();

    azimuthEncoder.updateValues(azimuthEncoderValues);

    steerMotor.setPosition(azimuthEncoderValues.angleRotations);
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    ShuffleboardLayout valuesLayout =
        tab.getLayout(this.config.name + " Values", BuiltInLayouts.kList);

    valuesLayout.addNumber(
        "Azimuth Encoder Absolute Angle (deg)",
        () -> Units.rotationsToDegrees(azimuthEncoderValues.angleRotations));
    valuesLayout.addNumber("Steer Motor Angle (deg)", () -> getState().angle.getDegrees());

    valuesLayout.addNumber(
        "Steer Motor Omega (dps)",
        () -> Units.rotationsToDegrees(steerMotorValues.omegaRotationsPerSecond));
    valuesLayout.addNumber("Drive Motor Velocity (mps)", () -> getState().speedMetersPerSecond);

    ShuffleboardLayout setpointLayout =
        tab.getLayout(this.config.name + " Setpoints", BuiltInLayouts.kList);

    setpointLayout.addBoolean("At Setpoint?", this::atSetpoint);

    setpointLayout.addDouble(
        "Steer Motor Setpoint (deg)",
        () -> Units.rotationsToDegrees(steerMotorSetpointAngleRotations));
    setpointLayout.addBoolean("Steer Motor At Setpoint?", this::atSteerMotorSetpoint);

    setpointLayout.addDouble(
        "Drive Motor Setpoint (mps)", () -> driveMotorSetpointVelocityMetersPerSecond);
    setpointLayout.addBoolean("Drive Motor At Setpoint?", this::atDriveMotorSetpoint);
  }

  @Override
  public void outputTelemetry() {}

  /** Updates values with sensor information. */
  public void update() {
    azimuthEncoder.updateValues(azimuthEncoderValues);

    steerMotor.updateValues(steerMotorValues);
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
            setpoint, Rotation2d.fromRotations(steerMotorValues.angleRotations));

    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/bf960378bca7fe3f32c46d3d529925d960d1ff37/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/Module.java#L117
    double steerMotorErrorRadians = Units.rotationsToRadians(getSteerMotorErrorRotations());
    setpoint.speedMetersPerSecond *= Math.cos(steerMotorErrorRadians);

    setSteerMotorSetpoint(setpoint.angle);
    setDriveMotorSetpoint(setpoint.speedMetersPerSecond);
  }

  private void setSteerMotorSetpoint(Rotation2d angle) {
    final double angleRotations = angle.getRotations();

    steerMotorSetpointAngleRotations = angleRotations;

    steerMotor.setSetpoint(steerMotorSetpointAngleRotations);
  }

  private void setDriveMotorSetpoint(double velocityMetersPerSecond) {
    driveMotorSetpointVelocityMetersPerSecond = velocityMetersPerSecond;

    driveMotor.setVelocitySetpoint(driveMotorSetpointVelocityMetersPerSecond);
  }

  private double getSteerMotorErrorRotations() {
    return steerMotorSetpointAngleRotations - steerMotorValues.angleRotations;
  }

  private double getDriveMotorErrorMetersPerSecond() {
    return driveMotorSetpointVelocityMetersPerSecond - driveMotorValues.velocityMetersPerSecond;
  }

  public boolean atSetpoint() {
    return atSteerMotorSetpoint() && atDriveMotorSetpoint();
  }

  private boolean atSteerMotorSetpoint() {
    return Math.abs(getSteerMotorErrorRotations()) <= Swerve.STEER_TOLERANCE;
  }

  private boolean atDriveMotorSetpoint() {
    return Math.abs(getDriveMotorErrorMetersPerSecond()) <= Swerve.DRIVE_TOLERANCE;
  }

  /**
   * Gets the state of the module.
   *
   * @return the state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }

  /**
   * Gets the module position relative to the initial position.
   *
   * @return the position relative to the initial position.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }
}
