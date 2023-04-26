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
    public double angleRadians;
    public double lengthMeters;

    public State() {}

    public State(double angleRadians, double lengthMeters) {
      this.angleRadians = angleRadians;
      this.lengthMeters = lengthMeters;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        boolean anglesEqual = this.angleRadians == rhs.angleRadians;
        boolean lengthsEqual = this.lengthMeters == rhs.lengthMeters;
        return anglesEqual && lengthsEqual;
      }
      return false;
    }

    @Override
    public int hashCode() {
      return Objects.hash(angleRadians, lengthMeters);
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

  public boolean isEnabled() {
    return enabled;
  }

  public void disable() {
    enabled = false;
  }

  public void enable() {
    enabled = true;
  }

  public LockType getLocked() {
    if (values.extensionBrakeIsActive && values.rotationBrakeIsActive) return LockType.kBoth;
    if (values.extensionBrakeIsActive) return LockType.kExtension;
    if (values.rotationBrakeIsActive) return LockType.kRotation;
    return LockType.kNeither;
  }

  public void lock(LockType type) {
    setLocked(type, true);
  }

  public void unlock(LockType type) {
    setLocked(type, false);
  }

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

  public boolean atGoal() {
    boolean extensionInTolerance =
        Math.abs(goal.lengthMeters - values.extensionLengthMeters)
            < Constants.Arm.Extension.TOLERANCE;
    boolean rotationInTolerance =
        Math.abs(goal.angleRadians - values.rotationAngleRadians)
            < Constants.Arm.Rotation.TOLERANCE;
    return extensionInTolerance && rotationInTolerance;
  }

  public State getGoal() {
    return goal;
  }

  public void setGoal(State state) {
    goal = state;
  }

  public boolean isReset() {
    return reset;
  }

  public void reset(State state) {
    reset = true;
    io.setExtensionPosition(state.lengthMeters);
    io.setRotationPosition(state.angleRadians);
  }

  public State getState() {
    return new State(values.rotationAngleRadians, values.extensionLengthMeters);
  }

  public void drive(double rotationPercent, double extensionPercent) {
    io.setRotationVoltage(rotationPercent * Constants.NOMINAL_VOLTAGE);
    io.setExtensionVoltage(extensionPercent * Constants.NOMINAL_VOLTAGE);
  }

  private void updateMechanism() {
    armMech2d.setLength(values.extensionLengthMeters * 20);
    armMech2d.setAngle(Math.toDegrees(values.rotationAngleRadians));

    if (getLocked() != LockType.kNeither) {
      armMech2d.setColor(new Color8Bit(255, 0, 0));
    } else {
      armMech2d.setColor(new Color8Bit(0, 255, 0));
    }
  }

  private void updateTelemetry() {
    SmartDashboard.putBoolean("extensionBrakeIsActive", values.extensionBrakeIsActive);
    SmartDashboard.putBoolean("rotationBrakeIsActive", values.rotationBrakeIsActive);
    SmartDashboard.putNumber("extensionLengthMeters", values.extensionLengthMeters);
    SmartDashboard.putNumber("rotationAngleRadians", values.rotationAngleRadians);
    SmartDashboard.putNumber("extensionLengthMetersGoal", goal.lengthMeters);
    SmartDashboard.putNumber("rotationAngleRadiansGoal", goal.angleRadians);
    SmartDashboard.putBoolean("atGoal", atGoal());
  }

  private void updateSetpoints() {
    io.setExtensionSetpoint(goal.lengthMeters);
    io.setRotationSetpoint(goal.angleRadians);
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    updateMechanism();
    updateTelemetry();

    if (isEnabled()) updateSetpoints();
  }
}
