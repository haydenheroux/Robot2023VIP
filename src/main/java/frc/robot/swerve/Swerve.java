package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;

public class Swerve extends SubsystemBase implements TelemetryOutputter {
  // Singleton instance
  private static Swerve instance = null;

  public final SwerveDriveKinematics kinematics;
  private final Module[] modules = new Module[4]; // FL=NW, FR=NE, BR=SE, BL=SW

  /** Creates a new Swerve. */
  private Swerve() {
    modules[0] = new Module(Constants.Swerve.NORTH_WEST);
    modules[1] = new Module(Constants.Swerve.NORTH_EAST);
    modules[2] = new Module(Constants.Swerve.SOUTH_EAST);
    modules[3] = new Module(Constants.Swerve.SOUTH_WEST);

    kinematics =
        new SwerveDriveKinematics(
            modules[0].config.location,
            modules[1].config.location,
            modules[2].config.location,
            modules[3].config.location);
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

    tab.addDoubleArray("Module States", () -> getStatesAsArray(getStates()));

    for (Module module : modules) {
      module.initializeDashboard();
    }
  }

  @Override
  public void outputTelemetry() {}

  public void setSetpoints(SwerveModuleState[] setpoints) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Swerve.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i]);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  public double[] getStatesAsArray(SwerveModuleState[] states) {
    double[] doubles = new double[8];

    for (int i = 0; i < 4; i++) {
      SwerveModuleState state = states[i];
      doubles[2 * i] = state.angle.getRadians();
      doubles[2 * i + 1] = state.speedMetersPerSecond;
    }

    return doubles;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }

    return positions;
  }
}
