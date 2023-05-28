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

public class Swerve extends SubsystemBase implements TelemetryOutputter {
  private static Swerve instance = null;

  private final Module[] modules = new Module[4];

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
      modules[i].setSetpoint(setpoints[i]);
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
  public SwerveModulePosition[] getArmPosition() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }

    return positions;
  }
  
  public Command orientModules(Rotation2d[] orientations) {
    return this.runOnce(() -> {
      setSetpoints(
        new SwerveModuleState[]{
          new SwerveModuleState(0.0, orientations[0]),
          new SwerveModuleState(0.0, orientations[1]),
          new SwerveModuleState(0.0, orientations[2]),
          new SwerveModuleState(0.0, orientations[3])
        }
      );
    });
  }

  public Command checkForwards() {
    return orientModules(new Rotation2d[]{
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0),
      Rotation2d.fromDegrees(0),
    });
  }

  public Command checkSideways() {
    return orientModules(new Rotation2d[]{
      Rotation2d.fromDegrees(90),
      Rotation2d.fromDegrees(90),
      Rotation2d.fromDegrees(90),
      Rotation2d.fromDegrees(90),
    });
  }

  public Command cross() {
    return orientModules(new Rotation2d[]{
      Rotation2d.fromDegrees(45),
      Rotation2d.fromDegrees(-45),
      Rotation2d.fromDegrees(45),
      Rotation2d.fromDegrees(-45),
    });
  }

}
