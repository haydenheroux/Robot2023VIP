// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.SuperstructureMechanism;
import frc.lib.telemetry.TelemetryOutputter;
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

  /** Creates a new Claw. */
  private Claw() {
    if (Robot.isSimulation()) {
      io = new ClawIOSim();
    } else {
      io = new ClawIOTalonSRX();
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
    return values.motorCurrentAmps >= Thresholds.THRESHOLD;
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

    SuperstructureMechanism.getInstance().updateClaw(state);
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
    stateOverridesLayout.add(
        this.runOnce(() -> state = State.kHolding).withName("Holding"));

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Motor Current (A)", () -> values.motorCurrentAmps);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
