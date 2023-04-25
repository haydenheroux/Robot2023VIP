// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public Rotation2d angle;
    public double length;

    public State() {}

    public State(Rotation2d angle, double length) {
      this.angle = angle;
      this.length = length;
    }

    public State(double degrees, double length) {
      this.angle = Rotation2d.fromDegrees(degrees);
      this.length = length;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        boolean anglesEqual = this.angle.equals(rhs.angle);
        boolean lengthsEqual = this.length == rhs.length;
        return anglesEqual && lengthsEqual;
      }
      return false;
    }

    @Override
    public int hashCode() {
      return Objects.hash(angle, length);
    }
  }

  // Singleton instance
  private static Arm instance = null;

  // ArmIO handler
  private final ArmIO io;
  private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

  /** Creates a new Arm. */
  private Arm() {
    if (Robot.isSimulation()) {
      io = new ArmIOSim();
    } else {
      io = null;
    }

    io.configure();
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public boolean isDisabled() {
    return false;
  }

  public void disable() {}

  public void enable() {}

  public LockType getLocked() {
    return LockType.kBoth;
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
    return true;
  }

  public State getGoal() {
    return new State();
  }

  public State getError() {
    return new State();
  }

  public boolean isReset() {
    return false;
  }

  public void reset(State state) {}

  public void drive(double anglePercent, double extensionPercent) {}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
