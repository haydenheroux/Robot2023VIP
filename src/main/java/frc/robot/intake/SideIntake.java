// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.SuperstructureMechanism;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
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

  // Moving average filter for bottomMotorCurrent
  private final LinearFilter bottomMotorCurrentFilter =
      LinearFilter.movingAverage(
          (int) (Constants.Intake.SideIntake.CURRENT_PERIOD * Constants.SAMPLES_PER_SECOND));
  private double filteredBottomMotorCurrentAmps = 0.0;

  // Moving average filter for topMotorCurrent
  private final LinearFilter topMotorCurrentFilter =
      LinearFilter.movingAverage(
          (int) (Constants.Intake.SideIntake.CURRENT_PERIOD * Constants.SAMPLES_PER_SECOND));
  private double filteredTopMotorCurrentAmps = 0.0;

  private State state = State.kDisabled;

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
    boolean isBottomHolding =
        filteredBottomMotorCurrentAmps >= Constants.Intake.SideIntake.BOTTOM_CURRENT_THRESHOLD;
    boolean isTopHolding =
        filteredTopMotorCurrentAmps >= Constants.Intake.SideIntake.TOP_CURRENT_THRESHOLD;
    return isBottomHolding && isTopHolding;
  }

  private double getBiasFor(double alphaRadians, double betaRadians) {
    double betaRelativeToAlphaRadians = (alphaRadians - Units.degreesToRadians(90.0)) + betaRadians;
    return getBiasFor(betaRelativeToAlphaRadians);
  }

  private double getBiasFor(double betaRelativeToAlphaRadians) {
    return Constants.Intake.SideIntake.RELATIVE_BIAS_FACTOR * Math.sin(betaRelativeToAlphaRadians);
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

  private void doAccept() {
    double bias =
        getBiasFor(
            Constants.Intake.SideIntake.MECHANISM_ANGLE, Constants.Intake.SideIntake.ACCEPT_ANGLE);
    double bottomMotorVoltage = bias + Constants.Intake.SideIntake.BASE_ACCEPTING_VOLTAGE;
    double topMotorVoltage = -bias + Constants.Intake.SideIntake.BASE_ACCEPTING_VOLTAGE;
    io.setBottomMotorVoltage(bottomMotorVoltage);
    io.setTopMotorVoltage(topMotorVoltage);
  }

  private void doEject() {
    double bias =
        getBiasFor(
            Constants.Intake.SideIntake.MECHANISM_ANGLE, Constants.Intake.SideIntake.EJECT_ANGLE);
    double bottomMotorVoltage = -bias + Constants.Intake.SideIntake.BASE_EJECTING_VOLTAGE;
    double topMotorVoltage = bias + Constants.Intake.SideIntake.BASE_EJECTING_VOLTAGE;
    io.setBottomMotorVoltage(bottomMotorVoltage);
    io.setTopMotorVoltage(topMotorVoltage);
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
        doAccept();
        break;
      case kDisabled:
        io.setBottomMotorDisabled();
        io.setTopMotorDisabled();
        break;
      case kEjecting:
        doEject();
        break;
      case kHolding:
        io.setBottomMotorVoltage(Constants.Intake.SideIntake.HOLDING_VOLTAGE);
        io.setTopMotorVoltage(Constants.Intake.SideIntake.HOLDING_VOLTAGE);
        break;
    }

    SuperstructureMechanism.getInstance().updateSideIntake(state);
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("State", state::toString);
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
