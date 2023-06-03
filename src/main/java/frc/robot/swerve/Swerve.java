package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryData;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;

/** Controls the four swerve modules used to drive the chassis. */
public class Swerve extends SubsystemBase implements TelemetryOutputter {
  private static Swerve instance = null;

  private final Module[] modules = new Module[4];

  /** Constructs a new swerve. */
  private Swerve() {
    modules[0] = new Module(Constants.Swerve.NORTH_WEST);
    modules[1] = new Module(Constants.Swerve.NORTH_EAST);
    modules[2] = new Module(Constants.Swerve.SOUTH_EAST);
    modules[3] = new Module(Constants.Swerve.SOUTH_WEST);
  }

  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  @Override
  public void periodic() {
    for (var module : modules) {
      module.update();
    }
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.addDoubleArray("Module States", () -> TelemetryData.asDoubleArray(getStates()));

    for (var module : modules) {
      module.initializeDashboard();
    }
  }

  @Override
  public void outputTelemetry() {}

  /**
   * Sets the module setpoints.
   *
   * <p>Mutates setpoints if some are unreachable.
   *
   * @param setpoints the module setpoints.
   */
  public void setSetpoints(SwerveModuleState[] setpoints) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Swerve.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i], false);
    }
  }

  /**
   * Sets the module setpoints.
   *
   * <p>Mutates setpoints if some are unreachable.
   *
   * @param setpoints the module setpoints.
   * @param force true if module should skip optimization.
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean force) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Swerve.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i], force);
    }
  }

  /**
   * Gets the state of each module.
   *
   * @return the state of each module.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  /**
   * Gets the position of each module relative to the initial position.
   *
   * @return the position relative to the initial position.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }

    return positions;
  }

  /**
   * Commands all swerve modules to orient themselves to an arbitrary orientation.
   *
   * @param orientations the orientation for each swerve module.
   * @return a command that orients all swerve modules to an arbitrary orientation.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return this.runOnce(
        () -> {
          setSetpoints(
              new SwerveModuleState[] {
                new SwerveModuleState(0.0, orientations[0]),
                new SwerveModuleState(0.0, orientations[1]),
                new SwerveModuleState(0.0, orientations[2]),
                new SwerveModuleState(0.0, orientations[3])
              },
              true);
        });
  }

  /**
   * Commands all swerve modules to orient themselves to zero degrees, or forwards. This is to check
   * if the swerve modules were calibrated so that zero degrees is facing forwards. If a swerve
   * module does not face forwards during this command, then that swerve module needs to have the
   * azimuth encoder recalibrated.
   *
   * @return a command that orients all swerve modules to zero degrees.
   */
  public Command checkForwards() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0),
        });
  }

  /**
   * Commands all swerve modules to orient themselves to ninety degrees, or one quarter rotation to
   * the left. This is to check if the swerve modules were calibrated so that ninety degrees is
   * facing to the left. If a swerve module does not face to the left during this command, then the
   * inversion configuration of that swerve module needs to be recalibrated.
   *
   * @return a command that orients all swerve modules to ninety degrees.
   */
  public Command checkSideways() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(90),
          Rotation2d.fromDegrees(90),
          Rotation2d.fromDegrees(90),
          Rotation2d.fromDegrees(90),
        });
  }

  /**
   * Commands all swerve modules to orient themselves towards the center, forming a cross shape.
   * This is to make the chassis more difficult to push, locking the chassis to one position unless
   * commanded to move.
   *
   * @return a command that orients all swerve modules towards the center.
   */
  public Command cross() {
    return orientModules(
        new Rotation2d[] {
          Rotation2d.fromDegrees(45),
          Rotation2d.fromDegrees(-45),
          Rotation2d.fromDegrees(45),
          Rotation2d.fromDegrees(-45),
        });
  }
}
