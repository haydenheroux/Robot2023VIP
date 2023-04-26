// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Objects;

public class Arm extends SubsystemBase {
  public enum LockType {
    kBoth,
    kExtension,
    kNeither,
    kRotation,
  }

  public static class State {
    public double extensionLengthMeters, rotationAngleRadians;

    public State() {}

    public State(double extensionLengthMeters, double rotationAngleRadians) {
      this.extensionLengthMeters = extensionLengthMeters;
      this.rotationAngleRadians = rotationAngleRadians;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        boolean extensionLengthsEqual = this.extensionLengthMeters == rhs.extensionLengthMeters;
        boolean rotationAnglesEqual = this.rotationAngleRadians == rhs.rotationAngleRadians;
        return extensionLengthsEqual && rotationAnglesEqual;
      }
      return false;
    }

    @Override
    public int hashCode() {
      return Objects.hash(extensionLengthMeters, rotationAngleRadians);
    }
  }

  // Singleton instance
  private static Arm instance = null;

  // ArmIO handler
  private final ArmIO io;
  private final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  private final Mechanism2d mech2d = new Mechanism2d(40, 40);
  private final MechanismRoot2d root = mech2d.getRoot("root", 20, 20);
  private final MechanismLigament2d armMech2d =
      root.append(
          new MechanismLigament2d(
              "Arm", values.extensionLengthMeters, Math.toDegrees(values.rotationAngleRadians)));

  private boolean enabled = false;

  private State goal = new State();

  private boolean reset = false;

  /** Creates a new Arm. */
  private Arm() {
    if (Robot.isSimulation()) {
      io = new ArmIOSim();
    } else {
      io = null;
    }

    io.configure();
    SmartDashboard.putData("Mechanism", mech2d);
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  /**
   * Returns whether the arm is under positional control.
   *
   * @return whether the arm is under positional control.
   */
  public boolean isEnabled() {
    return enabled;
  }

  /** Disables positional control. */
  public void disable() {
    enabled = false;
    io.setExtensionDisabled();
    io.setRotationDisabled();
  }

  /** Enables positional control. */
  public void enable() {
    enabled = true;
  }

  /**
   * Returns which brakes are active.
   *
   * @return which brakes are active.
   */
  public LockType getLocked() {
    if (values.extensionBrakeIsActive && values.rotationBrakeIsActive) return LockType.kBoth;
    if (values.extensionBrakeIsActive) return LockType.kExtension;
    if (values.rotationBrakeIsActive) return LockType.kRotation;
    return LockType.kNeither;
  }

  /**
   * Locks the specified brakes.
   *
   * @param type which brakes to lock.
   */
  public void lock(LockType type) {
    setLocked(type, true);
  }

  /**
   * Unlocks the specified brakes.
   *
   * @param type which brakes to unlock.
   */
  public void unlock(LockType type) {
    setLocked(type, false);
  }

  /**
   * Sets the specified brakes.
   *
   * @param type which brakes to set.
   * @param value what to set to.
   */
  private void setLocked(LockType type, boolean value) {
    switch (type) {
      case kBoth:
        io.setExtensionBrake(value);
        io.setRotationBrake(value);
      case kExtension:
        io.setExtensionBrake(value);
        break;
      case kNeither:
        break;
      case kRotation:
        io.setRotationBrake(value);
        break;
    }
  }

  /**
   * Returns if the error is in tolerance.
   *
   * @return if the error is in tolerance.
   */
  public boolean atGoal() {
    boolean extensionInTolerance =
        Math.abs(goal.extensionLengthMeters - values.extensionLengthMeters)
            < Constants.Arm.Extension.TOLERANCE;
    boolean rotationInTolerance =
        Math.abs(goal.rotationAngleRadians - values.rotationAngleRadians)
            < Constants.Arm.Rotation.TOLERANCE;
    return extensionInTolerance && rotationInTolerance;
  }

  /**
   * Returns the goal state of positional control.
   *
   * @return the goal state of positional control.
   */
  public State getGoal() {
    return goal;
  }

  /**
   * Sets the goal state for positional control.
   *
   * @param state the goal state.
   */
  public void setGoal(State state) {
    goal = state;
  }

  /**
   * Returns if the arm position has been reset.
   *
   * @return if the arm position has been reset.
   */
  public boolean isReset() {
    return reset;
  }

  /**
   * Resets the position of the arm to the state.
   *
   * @param state the state.
   */
  public void reset(State state) {
    reset = true;
    io.setExtensionPosition(state.extensionLengthMeters);
    io.setRotationPosition(state.rotationAngleRadians);
  }

  /**
   * Returns the current position of the arm.
   *
   * @return the current position of the arm.
   */
  public State getState() {
    return new State(values.rotationAngleRadians, values.extensionLengthMeters);
  }

  /**
   * Drives the arm with the specified speeds.
   *
   * @param extensionPercent speed to run extension motor at.
   * @param rotationPercent speed to run rotation motor at.
   */
  public void drive(double extensionPercent, double rotationPercent) {
    boolean extensionAtMin = values.extensionLengthMeters < Constants.Arm.Extension.MIN_LENGTH;
    boolean extensionIncreasing = extensionPercent > 0;
    boolean extensionAtMax = values.extensionLengthMeters > Constants.Arm.Extension.MAX_LENGTH;
    boolean extensionDecreasing = extensionPercent < 0;

    boolean extensionPastMin = extensionAtMin && extensionDecreasing;
    boolean extensionPastMax = extensionAtMax && extensionIncreasing;

    if (!extensionPastMin && !extensionPastMax) {
      io.setExtensionVoltage(extensionPercent * Constants.NOMINAL_VOLTAGE);
    }

    boolean rotationAtMin = values.rotationAngleRadians < Constants.Arm.Rotation.MIN_ANGLE;
    boolean rotationIncreasing = rotationPercent > 0;
    boolean rotationAtMax = values.rotationAngleRadians > Constants.Arm.Rotation.MAX_ANGLE;
    boolean rotationDecreasing = rotationPercent < 0;

    boolean rotationPastMin = rotationAtMin && rotationDecreasing;
    boolean rotationPastMax = rotationAtMax && rotationIncreasing;

    if (!rotationPastMin && !rotationPastMax) {
      io.setRotationVoltage(rotationPercent * Constants.NOMINAL_VOLTAGE);
    }
  }

  /** Updates the arm's mechanism representation with updated data. */
  private void updateMechanism() {
    armMech2d.setLength(values.extensionLengthMeters * 20);
    armMech2d.setAngle(Math.toDegrees(values.rotationAngleRadians));

    if (getLocked() != LockType.kNeither) {
      armMech2d.setColor(new Color8Bit(255, 0, 0));
    } else {
      armMech2d.setColor(new Color8Bit(0, 255, 0));
    }
  }

  /** Updates the arm's telemetry with updated data. */
  private void updateTelemetry() {
    SmartDashboard.putBoolean("extensionBrakeIsActive", values.extensionBrakeIsActive);
    SmartDashboard.putBoolean("rotationBrakeIsActive", values.rotationBrakeIsActive);
    SmartDashboard.putNumber("extensionLengthMeters", values.extensionLengthMeters);
    SmartDashboard.putNumber("rotationAngleRadians", values.rotationAngleRadians);
    SmartDashboard.putNumber("extensionLengthMetersGoal", goal.extensionLengthMeters);
    SmartDashboard.putNumber("rotationAngleRadiansGoal", goal.rotationAngleRadians);
    SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putBoolean("isEnabled", isEnabled());
  }

  /** Update's the arm's setpoints depending on the goal. */
  private void updateSetpoints() {
    io.setExtensionSetpoint(goal.extensionLengthMeters);
    io.setRotationSetpoint(goal.rotationAngleRadians);
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    updateMechanism();
    updateTelemetry();

    if (isEnabled()) updateSetpoints();
  }
}
