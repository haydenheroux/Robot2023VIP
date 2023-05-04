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
    boolean isBottomHolding = values.bottomMotorCurrentAmps >= Thresholds.BOTTOM_THRESHOLD;
    boolean isTopHolding = values.topMotorCurrentAmps >= Thresholds.TOP_THRESHOLD;
    return isBottomHolding && isTopHolding;
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
    double bias = SideIntakeMath.getBias();
    double bottomMotorVoltage = bias + Voltages.BASE_ACCEPTING;
    double topMotorVoltage = -bias + Voltages.BASE_ACCEPTING;
    io.setBottomMotorVoltage(bottomMotorVoltage);
    io.setTopMotorVoltage(topMotorVoltage);
  }

  private void doEject() {
    double bias = SideIntakeMath.getBias();
    double bottomMotorVoltage = -bias + Voltages.BASE_EJECTING;
    double topMotorVoltage = bias + Voltages.BASE_EJECTING;
    io.setBottomMotorVoltage(bottomMotorVoltage);
    io.setTopMotorVoltage(topMotorVoltage);
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
        io.setBottomMotorVoltage(Voltages.HOLDING);
        io.setTopMotorVoltage(Voltages.HOLDING);
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
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
