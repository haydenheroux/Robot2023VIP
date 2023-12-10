package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryData;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.swerve.ModuleConstants.ModuleLocation;

/** Controls the four swerve modules used to drive the chassis. */
public class Swerve extends SubsystemBase implements TelemetryOutputter {
  private static Swerve instance = null;

  private final ModuleIO[] modules = new ModuleIO[4];

  private final SwerveDriveKinematics kinematics;

  /** Constructs a new swerve. */
  private Swerve() {
    modules[0] = SwerveFactory.createModule(Constants.Swerve.NORTH_WEST); // FL
    modules[1] = SwerveFactory.createModule(Constants.Swerve.NORTH_EAST); // FR
    modules[2] = SwerveFactory.createModule(Constants.Swerve.SOUTH_EAST); // BR
    modules[3] = SwerveFactory.createModule(Constants.Swerve.SOUTH_WEST); // BL

    for (var module : modules) {
      module.configure();
    }

    kinematics =
        new SwerveDriveKinematics(
            ModuleLocation.of(true, true),
            ModuleLocation.of(true, false),
            ModuleLocation.of(false, false),
            ModuleLocation.of(false, true));
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
    tab.addDoubleArray("Module Setpoints", () -> TelemetryData.asDoubleArray(getSetpoints()));

    for (int i = 0; i < 4; i++) {
      ModuleIO module = modules[i];
      ShuffleboardLayout layout = tab.getLayout("Module " + i, BuiltInLayouts.kList).withSize(2, 4);

      layout.addNumber("Steer Motor Angle (deg)", () -> module.getState().angle.getDegrees());
      layout.addNumber("Drive Motor Velocity (mps)", () -> module.getState().speedMetersPerSecond);

      layout.addDouble("Steer Motor Setpoint (deg)", () -> module.getSetpoint().angle.getDegrees());
      layout.addDouble(
          "Drive Motor Setpoint (mps)", () -> module.getSetpoint().speedMetersPerSecond);
    }
  }

  @Override
  public void outputTelemetry() {}

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] setpoints = kinematics.toSwerveModuleStates(speeds);

    setSetpoints(setpoints, false);
  }

  /**
   * Sets the module setpoints.
   *
   * <p>Mutates setpoints if some are unreachable.
   *
   * @param setpoints the module setpoints.
   * @param force if true, do not optimize the setpoints.
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean force) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Physical.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i], force);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
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
   * Gets the setpoint of each module.
   *
   * @return the setpoint of each module.
   */
  public SwerveModuleState[] getSetpoints() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getSetpoint();
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
    return this.run(
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

  /**
   * Commands all swerve modules to orient themselves around the center.
   *
   * @return a command that orients all swerve modules around the center.
   */
  public Command holonomic() {
    return orientModules(new Rotation2d[] {
      Rotation2d.fromDegrees(-45),
      Rotation2d.fromDegrees(45),
      Rotation2d.fromDegrees(-45),
      Rotation2d.fromDegrees(45),
    });
  }
}
