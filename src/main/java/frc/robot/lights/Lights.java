package frc.robot.lights;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;

/** Controls the lights used to signal drivers, report errors, and play animations. */
public class Lights extends SubsystemBase implements TelemetryOutputter {
  private static Lights instance = null;

  private Color color = Color.kRed;

  /** Constructs a new lights subsystem. */
  private Lights() {}

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Lights");

    tab.addString("Main Color", () -> getColor().toString());
  }

  @Override
  public void outputTelemetry() {}

  /**
   * Sets the main color of the lights.
   *
   * @param color the main color of the lights.
   */
  public void setColor(Color color) {
    this.color = color;
  }

  /**
   * Gets the main color of the lights.
   *
   * @return the main color of the lights.
   */
  public Color getColor() {
    // TODO
    return color;
  }
}
