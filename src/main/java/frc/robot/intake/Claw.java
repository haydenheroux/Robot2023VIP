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
  public boolean isHoldingCone() {
    return values.motorCurrentAmps >= Constants.Intake.Claw.Thresholds.CONE_THRESHOLD;
  }

  /**
   * Returns if the claw is currently holding a cube.
   *
   * @return if the claw is currently holding a cube.
   */
  public boolean isHoldingCube() {
    return values.motorCurrentAmps >= Constants.Intake.Claw.Thresholds.CUBE_THRESHOLD;
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

  @Override
  public void periodic() {
    io.updateValues(values);

    if (state != State.kDisabled && state != State.kEjecting) {
      if (isHoldingCone()) {
        state = State.kHoldingCone;
      } else if (isHoldingCube()) {
        state = State.kHoldingCube;
      }
    }

    switch (state) {
      case kAccepting:
        io.setMotorVoltage(Constants.Intake.Claw.Voltages.ACCEPTING);
        break;
      case kDisabled:
        io.setMotorDisabled();
        break;
      case kEjecting:
        io.setMotorVoltage(Constants.Intake.Claw.Voltages.EJECTING);
        break;
      case kHoldingCone:
        io.setMotorVoltage(Constants.Intake.Claw.Voltages.HOLDING_CONE);
        break;
      case kHoldingCube:
        io.setMotorVoltage(Constants.Intake.Claw.Voltages.HOLDING_CUBE);
        break;
    }

    SuperstructureMechanism.getInstance().updateClaw(state);
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("State", state::toString);
    tab.addBoolean("Is Holding Cone?", this::isHoldingCone);
    tab.addBoolean("Is Holding Cube?", this::isHoldingCube);

    ShuffleboardLayout stateOverridesLayout =
        tab.getLayout("State Overrides", BuiltInLayouts.kList);
    stateOverridesLayout.add(this.runOnce(() -> state = State.kAccepting).withName("Accepting"));
    stateOverridesLayout.add(this.runOnce(() -> state = State.kDisabled).withName("Disabled"));
    stateOverridesLayout.add(this.runOnce(() -> state = State.kEjecting).withName("Ejecting"));
    stateOverridesLayout.add(
        this.runOnce(() -> state = State.kHoldingCone).withName("Holding Cone"));
    stateOverridesLayout.add(
        this.runOnce(() -> state = State.kHoldingCube).withName("Holding Cube"));

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Motor Current (A)", () -> values.motorCurrentAmps);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
