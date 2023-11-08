package frc.robot.lights;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Robot;
import frc.robot.lights.CANdleIO.CANdleIOValues;

/** Controls the lights used to signal drivers, report errors, and play animations. */
public class Lights extends SubsystemBase implements TelemetryOutputter {
  private static Lights instance = null;

  private final CANdleIO candle;
  private final CANdleIOValues candleValues = new CANdleIOValues();

  /** Constructs a new lights subsystem. */
  private Lights() {
    if (Robot.isSimulation()) {
      candle = new CANdleIOSim();
    } else {
      candle = new CANdleIOSim();
    }
  }

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }

    return instance;
  }

  @Override
  public void periodic() {
    candle.updateValues(candleValues);
  }

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
    candle.setColor(color);
  }

  /**
   * Gets the main color of the lights.
   *
   * @return the main color of the lights.
   */
  public Color getColor() {
    return new Color(candleValues.red, candleValues.green, candleValues.blue);
  }
}
