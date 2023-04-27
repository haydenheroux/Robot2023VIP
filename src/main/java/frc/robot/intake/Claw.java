// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Claw extends SubsystemBase {
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
   * @return if the claw is currently holding a cone.
   */
  public boolean isHoldingCone() {
    if (state == State.kAccepting) {
      // If we are unsure if the cone has been accepted, test the current
      return filteredMotorCurrentAmps >= Constants.Intake.Claw.CONE_CURRENT_THRESHOLD;
    }
    // Returns true if already holding cone
    return state == State.kHoldingCone;
  }

  /**
   * Returns if the claw is currently holding a cube.
   * @return if the claw is currently holding a cube.
   */
  public boolean isHoldingCube() {
    if (state == State.kAccepting) {
      // If we are unsure if the cube has been accepted, test the current
      return filteredMotorCurrentAmps >= Constants.Intake.Claw.CUBE_CURRENT_THRESHOLD;
    }
    // Returns true if already holding cube
    return state == State.kHoldingCube;
  }

  public void accept() {
    setState(State.kAccepting);
  }

  public void disable() {
    setState(State.kDisabled);
  }

  public void eject() {
    setState(State.kEjecting);
  }

  public void holdOrDisable() {
    if (state == State.kHoldingCone || state == State.kHoldingCube) {
      // If already holding, keep holding
      return;
    }
    // Otherwise (if accepting and not holding, or ejecting), disable
    disable();
  }

  public void setState(State state) {
    this.state = state;
  }

  public void updateTelemetry() {
    SmartDashboard.putString("state", state.toString());
    SmartDashboard.putNumber("motorCurrentAmps", values.motorCurrentAmps);
    SmartDashboard.putNumber("estimatedMotorCurrentAmps", filteredMotorCurrentAmps);
    SmartDashboard.putBoolean("isHoldingCone", isHoldingCone());
    SmartDashboard.putBoolean("isHoldingCube", isHoldingCube());
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    filteredMotorCurrentAmps = motorCurrentFilter.calculate(values.motorCurrentAmps);

    updateTelemetry();

    if (state == State.kAccepting) {
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
}
