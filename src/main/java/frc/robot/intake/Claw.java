package frc.robot.intake;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.Mechanisms;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Constants.Intake.Claw.Thresholds;
import frc.robot.Constants.Intake.Claw.Voltages;
import frc.robot.Robot;

public class Claw extends SubsystemBase implements TelemetryOutputter {
  public enum State {
    kAccepting,
    kDisabled,
    kEjecting,
    kHolding,
  }

  // Singleton instance
  private static Claw instance = null;

  // ClawIO handler
  private final ClawIO io;
  private final ClawIO.ClawIOValues values = new ClawIO.ClawIOValues();

  private State state = State.kDisabled;
  private MedianFilter motorCurrentFilter =
      new MedianFilter((int) (Constants.ITERATIONS_PER_SECOND * Thresholds.PERIOD));
  private double filteredMotorCurrentAmps = 0.0;

  /** Creates a new Claw. */
  private Claw() {
    if (Robot.isSimulation()) {
      io = new ClawIOSim();
    } else {
      io = new ClawIOSim();
    }

    io.configure();
  }

  public static Claw getInstance() {
    if (instance == null) {
      instance = new Claw();
    }
    return instance;
  }

  /**
   * Returns if the claw is currently holding a cone.
   *
   * @return if the claw is currently holding a cone.
   */
  public boolean isHolding() {
    return filteredMotorCurrentAmps >= Thresholds.THRESHOLD;
  }

  public Command accept() {
    return this.runOnce(() -> state = State.kAccepting);
  }

  public Command disable() {
    return this.runOnce(() -> state = State.kDisabled);
  }

  public Command eject() {
    return this.runOnce(() -> state = State.kEjecting);
  }

  public Command holdOrDisable() {
    return this.runOnce(
        () -> {
          if (state != State.kHolding) {
            state = State.kDisabled;
          }
        });
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    filteredMotorCurrentAmps = motorCurrentFilter.calculate(values.motorCurrentAmps);

    if (state != State.kDisabled && state != State.kEjecting) {
      if (isHolding()) {
        state = State.kHolding;
      }
    }

    switch (state) {
      case kAccepting:
        io.setMotorVoltage(Voltages.ACCEPTING);
        break;
      case kDisabled:
        io.setMotorDisabled();
        break;
      case kEjecting:
        io.setMotorVoltage(Voltages.EJECTING);
        break;
      case kHolding:
        io.setMotorVoltage(Voltages.HOLDING);
        break;
    }

    Mechanisms.getInstance().updateClaw(state);
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("State", () -> state.toString());
    tab.addBoolean("Is Holding?", this::isHolding);

    ShuffleboardLayout stateOverridesLayout =
        tab.getLayout("State Overrides", BuiltInLayouts.kList);
    stateOverridesLayout.add(this.runOnce(() -> state = State.kAccepting).withName("Accepting"));
    stateOverridesLayout.add(this.runOnce(() -> state = State.kDisabled).withName("Disabled"));
    stateOverridesLayout.add(this.runOnce(() -> state = State.kEjecting).withName("Ejecting"));
    stateOverridesLayout.add(this.runOnce(() -> state = State.kHolding).withName("Holding"));

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Motor Current (A)", () -> values.motorCurrentAmps);

    ShuffleboardLayout filteredValuesLayout =
        tab.getLayout("Filtered Values", BuiltInLayouts.kList);
    filteredValuesLayout.addNumber("Filtered Motor Current (A)", () -> filteredMotorCurrentAmps);
  }

  @Override
  public void outputTelemetry() {}
}
