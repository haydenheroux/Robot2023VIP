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
import frc.robot.Constants.Intake.SideIntake.Thresholds;
import frc.robot.Constants.Intake.SideIntake.Voltages;
import frc.robot.Robot;

public class SideIntake extends SubsystemBase implements TelemetryOutputter {
  public enum State {
    kAccepting,
    kDisabled,
    kEjecting,
    kHolding,
  }

  // Singleton instance
  private static SideIntake instance = null;

  // SideIntakeIO handler
  private final SideIntakeIO io;
  private final SideIntakeIO.SideIntakeIOValues values = new SideIntakeIO.SideIntakeIOValues();

  private State state = State.kDisabled;

  private MedianFilter bottomMotorCurrentFilter =
      new MedianFilter((int) (Constants.ITERATIONS_PER_SECOND * Thresholds.PERIOD));
  private double filteredBottomMotorCurrentAmps = 0.0;

  private MedianFilter topMotorCurrentFilter =
      new MedianFilter((int) (Constants.ITERATIONS_PER_SECOND * Thresholds.PERIOD));
  private double filteredTopMotorCurrentAmps = 0.0;

  /** Creates a new SideIntake. */
  private SideIntake() {
    if (Robot.isSimulation()) {
      io = new SideIntakeIOSim();
    } else {
      io = new SideIntakeIOTalonSRX();
    }

    io.configure();
  }

  public static SideIntake getInstance() {
    if (instance == null) {
      instance = new SideIntake();
    }
    return instance;
  }

  public boolean isHolding() {
    boolean isBottomHolding = filteredBottomMotorCurrentAmps >= Thresholds.BOTTOM_THRESHOLD;
    boolean isTopHolding = filteredTopMotorCurrentAmps >= Thresholds.TOP_THRESHOLD;
    return isBottomHolding || isTopHolding;
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

    filteredBottomMotorCurrentAmps =
        bottomMotorCurrentFilter.calculate(values.bottomMotorCurrentAmps);
    filteredTopMotorCurrentAmps = topMotorCurrentFilter.calculate(values.topMotorCurrentAmps);

    if (state != State.kDisabled && state != State.kEjecting) {
      if (isHolding()) {
        state = State.kHolding;
      }
    }

    switch (state) {
      case kAccepting:
        io.setBottomMotorVoltage(Voltages.ACCEPTING);
        io.setTopMotorVoltage(Voltages.ACCEPTING);
        break;
      case kDisabled:
        io.setBottomMotorDisabled();
        io.setTopMotorDisabled();
        break;
      case kEjecting:
        io.setBottomMotorVoltage(Voltages.EJECTING);
        io.setTopMotorVoltage(Voltages.EJECTING);
        break;
      case kHolding:
        io.setBottomMotorVoltage(Voltages.HOLDING);
        io.setTopMotorVoltage(Voltages.HOLDING);
        break;
    }

    Mechanisms.getInstance().updateSideIntake(state);
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
    valuesLayout.addNumber("Bottom Motor Current (A)", () -> values.bottomMotorCurrentAmps);
    valuesLayout.addNumber("Top Motor Current (A)", () -> values.topMotorCurrentAmps);

    ShuffleboardLayout filteredValuesLayout =
        tab.getLayout("Filtered Values", BuiltInLayouts.kList);
    filteredValuesLayout.addNumber(
        "Filtered Bottom Motor Current (A)", () -> filteredBottomMotorCurrentAmps);
    filteredValuesLayout.addNumber(
        "Filtered Top Motor Current (A)", () -> filteredTopMotorCurrentAmps);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
