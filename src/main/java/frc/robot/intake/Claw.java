// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;

public class Claw extends SubsystemBase implements TelemetryOutputter {
  public enum State {
    kAccepting,
    kDisabled,
    kEjecting,
    kHoldingCone,
    kHoldingCube,
  }

  // Singleton instance
  private static Claw instance = null;

  // ClawIO handler
  private final ClawIO io;
  private final ClawIO.ClawIOValues values = new ClawIO.ClawIOValues();

  // Moving average filter for motorCurrent
  private final LinearFilter motorCurrentFilter =
      LinearFilter.movingAverage(
          (int) (Constants.Intake.Claw.CURRENT_PERIOD * Constants.SAMPLES_PER_SECOND));
  private double filteredMotorCurrentAmps = 0.0;

  private State state = State.kDisabled;

  /** Creates a new Claw. */
  private Claw() {
    if (Robot.isSimulation()) {
      io = new ClawIOSim();
    } else {
      io = null;
    }
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
  public boolean isHoldingCone() {
    return filteredMotorCurrentAmps >= Constants.Intake.Claw.CONE_CURRENT_THRESHOLD;
  }

  /**
   * Returns if the claw is currently holding a cube.
   *
   * @return if the claw is currently holding a cube.
   */
  public boolean isHoldingCube() {
    return filteredMotorCurrentAmps >= Constants.Intake.Claw.CUBE_CURRENT_THRESHOLD;
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
          if (state != State.kHoldingCone && state != State.kHoldingCube) {
            state = State.kDisabled;
          }
        });
  }

  public void updateTelemetry() {}

  @Override
  public void periodic() {
    io.updateValues(values);

    filteredMotorCurrentAmps = motorCurrentFilter.calculate(values.motorCurrentAmps);

    updateTelemetry();

    if (state != State.kDisabled && state != State.kEjecting) {
      if (isHoldingCone()) {
        state = State.kHoldingCone;
      } else if (isHoldingCube()) {
        state = State.kHoldingCube;
      }
    }

    switch (state) {
      case kAccepting:
        io.setMotorVoltage(Constants.Intake.Claw.ACCEPTING_VOLTAGE);
        break;
      case kDisabled:
        io.setMotorDisabled();
        break;
      case kEjecting:
        io.setMotorVoltage(Constants.Intake.Claw.EJECTING_VOLTAGE);
        break;
      case kHoldingCone:
        io.setMotorVoltage(Constants.Intake.Claw.HOLDING_CONE_VOLTAGE);
        break;
      case kHoldingCube:
        io.setMotorVoltage(Constants.Intake.Claw.HOLDING_CUBE_VOLTAGE);
        break;
    }
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("State", state::toString);
    tab.addBoolean("Is Holding Cone?", this::isHoldingCone);
    tab.addBoolean("Is Holding Cube?", this::isHoldingCube);

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Motor Current (A)", () -> values.motorCurrentAmps);

    ShuffleboardLayout filteredValuesLayout =
        tab.getLayout("Filtered Values", BuiltInLayouts.kList);
    filteredValuesLayout.addNumber("Filtered Motor Current (A)", () -> filteredMotorCurrentAmps);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
