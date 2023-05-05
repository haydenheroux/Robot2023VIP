// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.SuperstructureMechanism;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Positions;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Robot;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase implements TelemetryOutputter {

  public static class State {

    public final double extensionLengthMeters;
    public final Rotation2d rotationAngle;

    public State(double extensionLengthMeters, Rotation2d rotationAngle) {
      this.extensionLengthMeters = extensionLengthMeters;
      this.rotationAngle = rotationAngle;
    }

    public static State fromPosition(ArmPosition position) {
      double extensionLengthMeters = position.getNorm() - Extension.LENGTH_OFFSET;
      return new State(extensionLengthMeters, position.getAngle());
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        boolean extensionLengthsEqual = this.extensionLengthMeters == rhs.extensionLengthMeters;
        boolean rotationAnglesEqual = this.rotationAngle == rhs.rotationAngle;
        return extensionLengthsEqual && rotationAnglesEqual;
      }
      return false;
    }

    @Override
    public int hashCode() {
      return Objects.hash(extensionLengthMeters, rotationAngle);
    }
  }

  public enum Type {
    kBoth,
    kExtension,
    kNeither,
    kRotation,
  }

  // Singleton instance
  private static Arm instance = null;

  // ArmIO handler
  private final ArmIO io;
  private final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  private ArmPosition position = Positions.STOW;
  private ArmPosition goal = position;
  private ArmPosition setpoint = goal;

  /** Creates a new Arm. */
  private Arm() {
    if (Robot.isSimulation()) {
      io = new ArmIOSim();
    } else {
      io = new ArmIOTalonFXPID();
    }

    io.configure();

    State state = State.fromPosition(position);
    io.setExtensionPosition(state.extensionLengthMeters);
    io.setRotationPosition(state.rotationAngle.getRadians());
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public ArmPosition getPosition() {
    return position;
  }

  public boolean isIntersectingGrid() {
    return ArmKinematics.isIntersectingGrid(position);
  }

  public boolean isWithinRuleZone() {
    return ArmKinematics.isWithinRuleZone(position);
  }

  public void disable(Type type) {
    switch (type) {
      case kBoth:
        io.setExtensionDisabled();
        io.setRotationDisabled();
        break;
      case kExtension:
        io.setExtensionDisabled();
        break;
      case kNeither:
        break;
      case kRotation:
        io.setRotationDisabled();
        break;
    }
  }

  public void setVoltage(Type type, double volts) {
    switch (type) {
      case kBoth:
        io.setExtensionVoltage(volts);
        io.setRotationVoltage(volts);
        break;
      case kExtension:
        io.setExtensionVoltage(volts);
        break;
      case kNeither:
        break;
      case kRotation:
        io.setRotationVoltage(volts);
        break;
    }
  }

  /**
   * Returns which brakes are active.
   *
   * @return which brakes are active.
   */
  public Type getLocked() {
    if (values.extensionBrakeIsActive && values.rotationBrakeIsActive) return Type.kBoth;
    if (values.extensionBrakeIsActive) return Type.kExtension;
    if (values.rotationBrakeIsActive) return Type.kRotation;
    return Type.kNeither;
  }

  /**
   * Locks the specified brakes.
   *
   * @param type which brakes to lock.
   */
  public void lock(Type type) {
    setLocked(type, true);
  }

  /**
   * Unlocks the specified brakes.
   *
   * @param type which brakes to unlock.
   */
  public void unlock(Type type) {
    setLocked(type, false);
  }

  /**
   * Sets the specified brakes.
   *
   * @param type which brakes to set.
   * @param value what to set to.
   */
  private void setLocked(Type type, boolean value) {
    switch (type) {
      case kBoth:
        io.setExtensionBrake(value);
        io.setRotationBrake(value);
        break;
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

  public boolean extensionIsAtMax() {
    return values.extensionLengthMeters > Extension.MAX_LENGTH;
  }

  public boolean extensionIsAtMin() {
    return values.extensionLengthMeters < Extension.MIN_LENGTH;
  }

  public boolean rotationIsAtMax() {
    return values.rotationAngleRadians > Rotation.MAX_ANGLE.getRadians();
  }

  public boolean rotationIsAtMin() {
    return values.rotationAngleRadians < Rotation.MIN_ANGLE.getRadians();
  }

  public boolean at(ArmPosition position) {
    return this.position.at(position);
  }

  public void setGoal(ArmPosition goal) {
    this.goal = goal;
  }

  public void setSetpoint(ArmPosition setpoint) {
    this.setpoint = setpoint;

    State state = State.fromPosition(setpoint);

    io.setExtensionSetpoint(state.extensionLengthMeters);
    io.setRotationSetpoint(state.rotationAngle.getRadians());
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    position =
        ArmPosition.fromState(
            new Arm.State(
                values.extensionLengthMeters, Rotation2d.fromRadians(values.rotationAngleRadians)));

    SuperstructureMechanism.getInstance().updateArm(position, getLocked());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("Is Locked?", () -> getLocked().toString());
    tab.add(this.runUnlock(Type.kBoth).withName("Unlock Both"));
    tab.add(this.runLock(Type.kBoth).withName("Lock Both"));

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Extension Length (m)", () -> values.extensionLengthMeters);
    valuesLayout.addNumber(
        "Rotation Angle (deg)", () -> Units.radiansToDegrees(values.rotationAngleRadians));
    valuesLayout.addBoolean("Extension Brake Is Active?", () -> values.extensionBrakeIsActive);
    valuesLayout.addBoolean("Rotation Brake Is Active?", () -> values.rotationBrakeIsActive);

    ShuffleboardLayout positionLayout = tab.getLayout("Position", BuiltInLayouts.kList);
    positionLayout.addNumber("Arm Length (m)", () -> position.getNorm());
    positionLayout.addNumber("Arm Angle (deg)", () -> position.getAngle().getDegrees());

    ShuffleboardLayout goalLayout = tab.getLayout("Goal", BuiltInLayouts.kList);
    goalLayout.addNumber("Arm Length Goal (m)", () -> goal.getNorm());
    goalLayout.addNumber("Arm Angle Goal (deg)", () -> goal.getAngle().getDegrees());
    goalLayout.addBoolean("At Goal?", () -> at(goal));

    ShuffleboardLayout setpointLayout = tab.getLayout("Setpoint", BuiltInLayouts.kList);
    setpointLayout.addNumber("Arm Length Setpoint (m)", () -> setpoint.getNorm());
    setpointLayout.addNumber("Arm Angle Setpoint (deg)", () -> setpoint.getAngle().getDegrees());
    setpointLayout.addBoolean("At Setpoint?", () -> at(setpoint));
  }

  @Override
  public void outputTelemetry() {}

  public Command runLock(Type type) {
    return this.runOnce(() -> this.lock(type));
  }

  public Command runUnlock(Type type) {
    return this.runOnce(() -> this.unlock(type));
  }

  public Command runToGoal(ArmPosition goal) {
    return new ToGoal(this, goal);
  }

  public Command runManualExtension(DoubleSupplier percentSupplier) {
    return new ManualExtension(this, percentSupplier);
  }

  public Command runManualRotation(DoubleSupplier percentSupplier) {
    return new ManualRotation(this, percentSupplier);
  }
}
