package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;
import frc.robot.swerve.SteerMotorIO.SteerMotorValues;
import java.util.function.Consumer;
import java.util.function.Supplier;

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

    ShuffleboardLayout layout = tab.getLayout(this.config.name, BuiltInLayouts.kList);
    layout.addNumber(
        "Absolute Angle (deg)",
        () -> Units.rotationsToDegrees(azimuthEncoderValues.angleRotations));
    layout.addNumber("Angle (deg)", () -> getState().angle.getDegrees());
    layout.addNumber(
        "Omega (dps)", () -> Units.rotationsToDegrees(steerMotorValues.omegaRotationsPerSecond));
    layout.addNumber("Velocity (mps)", () -> getState().speedMetersPerSecond);
    layout.add(spinSteerMotor(Rotation2d.fromDegrees(1), 0.5).withName("Fine Steer"));
    layout.add(spinSteerMotor(Rotation2d.fromDegrees(20), 0.25).withName("Coarse Steer"));
    layout.add(spinDriveMotor(true).withName("Drive Forwards"));
    layout.add(spinDriveMotor(false).withName("Drive Backwards"));
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

    driveMotor.setVelocitySetpoint(setpoint.speedMetersPerSecond);

    if (force == false) {
      SwerveMath.dejitter(setpoint, Rotation2d.fromRotations(steerMotorValues.angleRotations));
    }

    steerMotor.setSetpoint(setpoint.angle.getRotations());

    if (true) {
      SmartDashboard.putNumber(config.name + "/angle", setpoint.angle.getDegrees());
      SmartDashboard.putNumber(config.name + "/speed", setpoint.speedMetersPerSecond);
    }
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

  public Command spinSteerMotor(Rotation2d stride, double timeout) {
    Supplier<Rotation2d> nextSetpoint = () -> getState().angle.plus(stride);

    Consumer<Rotation2d> setSteerSetpoint =
        rotation -> steerMotor.setSetpoint(rotation.getRotations());

    Command toNextSetpoint =
        Commands.run(() -> setSteerSetpoint.accept(nextSetpoint.get()), Swerve.getInstance())
            .raceWith(Commands.waitSeconds(timeout));

    return toNextSetpoint.repeatedly().finallyDo(interrupted -> setSteerSetpoint.accept(getState().angle));
  }

  public Command spinDriveMotor(boolean forwards) {
    final double velocityMetersPerSecond = forwards ? Constants.Swerve.MAX_SPEED : -Constants.Swerve.MAX_SPEED;

    Command toFullSpeed = Commands.run(() -> driveMotor.setVelocitySetpoint(velocityMetersPerSecond), Swerve.getInstance());

    return toFullSpeed.finallyDo(interrupted -> driveMotor.setVelocitySetpoint(0.0));
  }
}
