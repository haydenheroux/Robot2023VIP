// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.SuperstructureMechanism;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase implements TelemetryOutputter {

  public static class State {

    public final double extensionLengthMeters, rotationAngleRadians;

    public State(double extensionLengthMeters, double rotationAngleRadians) {
      this.extensionLengthMeters = extensionLengthMeters;
      this.rotationAngleRadians = rotationAngleRadians;
    }

    public static State fromPosition(ArmPosition position) {
      double extensionLengthMeters =
          position.getLengthMeters() - Constants.Arm.Extension.LENGTH_OFFSET;
      double rotationAngleRadians = position.getAngleRadians();
      return new State(extensionLengthMeters, rotationAngleRadians);
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

  public enum LockType {
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

  private boolean enabled = false;

  private ArmPosition position = Constants.Arm.Positions.STOW;
  private ArmPosition goal = Constants.Arm.Positions.STOW;
  private ArmTrajectory trajectory = new ArmTrajectory(position, goal);

  /** Creates a new Arm. */
  private Arm() {
    if (Robot.isSimulation()) {
      io = new ArmIOSim();
    } else {
      io = new ArmIOTalonFXPID();
    }

    io.configure();
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
  public Command disable() {
    return this.runOnce(
        () -> {
          enabled = false;
          io.setExtensionDisabled();
          io.setRotationDisabled();
        });
  }

  /** Enables positional control. */
  public Command enable() {
    return this.runOnce(
        () -> {
          enabled = true;
        });
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
  public Command lock(LockType type) {
    return setLocked(type, true);
  }

  /**
   * Unlocks the specified brakes.
   *
   * @param type which brakes to unlock.
   */
  public Command unlock(LockType type) {
    return setLocked(type, false);
  }

  /**
   * Sets the specified brakes.
   *
   * @param type which brakes to set.
   * @param value what to set to.
   */
  private Command setLocked(LockType type, boolean value) {
    return this.runOnce(
        () -> {
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
        });
  }

  /**
   * Returns if the error is in tolerance.
   *
   * @return if the error is in tolerance.
   */
  public boolean atGoal() {
    return position.at(goal);
  }

  /**
   * Returns the goal state of positional control.
   *
   * @return the goal state of positional control.
   */
  public ArmPosition getGoal() {
    return goal;
  }

  /**
   * Sets the goal state for positional control.
   *
   * @param goal the goal state.
   */
  public Command setGoal(ArmPosition goal) {
    return this.runOnce(
        () -> {
          this.goal = goal;
          this.trajectory = new ArmTrajectory(position, goal);
        });
  }

  public Command toGoal() {
    return this.unlock(LockType.kBoth)
        .andThen(this.enable())
        .andThen(Commands.waitUntil(this::atGoal))
        .finallyDo(
            interrupted -> {
              this.disable().andThen(this.lock(LockType.kBoth)).schedule();
            });
  }

  /**
   * Drives arm extension with the specified speed.
   *
   * @param percent speed to run extension motor at.
   */
  public Command driveExtension(DoubleSupplier percent) {
    return this.run(
        () -> {
          boolean extensionAtMin =
              values.extensionLengthMeters < Constants.Arm.Extension.MIN_LENGTH;
          boolean extensionIncreasing = -percent.getAsDouble() > 0;
          boolean extensionAtMax =
              values.extensionLengthMeters > Constants.Arm.Extension.MAX_LENGTH;
          boolean extensionDecreasing = -percent.getAsDouble() < 0;

          boolean extensionPastMin = extensionAtMin && extensionDecreasing;
          boolean extensionPastMax = extensionAtMax && extensionIncreasing;

          boolean isWithinRuleZone = ArmKinematics.isWithinRuleZone(position);
          boolean isLeavingRuleZone = !isWithinRuleZone && extensionIncreasing;

          boolean isAvoidingGrid = ArmKinematics.isAvoidingGrid(position);
          boolean isHittingGrid = !isAvoidingGrid && extensionIncreasing;

          if (!extensionPastMin && !extensionPastMax && !isLeavingRuleZone && !isHittingGrid) {
            io.setExtensionVoltage(-percent.getAsDouble() * Constants.NOMINAL_VOLTAGE);
          }
        });
  }

  /**
   * Drives arm rotation with the specified speed.
   *
   * @param percent speed to run rotation motor at.
   */
  public Command driveRotation(DoubleSupplier percent) {
    return this.run(
        () -> {
          boolean rotationAtMin = values.rotationAngleRadians < Constants.Arm.Rotation.MIN_ANGLE;
          boolean rotationIncreasing = -percent.getAsDouble() > 0;
          boolean rotationAtMax = values.rotationAngleRadians > Constants.Arm.Rotation.MAX_ANGLE;
          boolean rotationDecreasing = -percent.getAsDouble() < 0;

          boolean rotationPastMin = rotationAtMin && rotationDecreasing;
          boolean rotationPastMax = rotationAtMax && rotationIncreasing;

          if (!rotationPastMin && !rotationPastMax) {
            io.setRotationVoltage(-percent.getAsDouble() * Constants.NOMINAL_VOLTAGE);
          }
        });
  }

  /** Update's the arm's setpoints depending on the goal. */
  private void updateSetpoint() {
    if (position.at(trajectory.get())) trajectory.next();

    State setpoint = State.fromPosition(trajectory.get());

    io.setExtensionSetpoint(setpoint.extensionLengthMeters);
    io.setRotationSetpoint(setpoint.rotationAngleRadians);
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    position =
        ArmPosition.fromState(
            new Arm.State(values.extensionLengthMeters, values.rotationAngleRadians));

    if (isEnabled()) updateSetpoint();

    SuperstructureMechanism.getInstance().updateArm(position, getLocked());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("Is Locked?", () -> getLocked().toString());
    tab.add(this.unlock(LockType.kBoth).withName("Unlock Both"));
    tab.add(this.lock(LockType.kBoth).withName("Lock Both"));

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Extension Length (m)", () -> values.extensionLengthMeters);
    valuesLayout.addNumber(
        "Rotation Angle (deg)", () -> Units.radiansToDegrees(values.rotationAngleRadians));
    valuesLayout.addBoolean("Extension Brake Is Active?", () -> values.extensionBrakeIsActive);
    valuesLayout.addBoolean("Rotation Brake Is Active?", () -> values.rotationBrakeIsActive);

    ShuffleboardLayout positionLayout = tab.getLayout("Position", BuiltInLayouts.kList);
    positionLayout.addNumber("Arm Length (m)", () -> position.getLengthMeters());
    positionLayout.addNumber(
        "Arm Angle (deg)", () -> Units.radiansToDegrees(position.getAngleRadians()));

    ShuffleboardLayout goalLayout = tab.getLayout("Goal", BuiltInLayouts.kList);
    goalLayout.addNumber("Arm Length Goal (m)", () -> goal.getLengthMeters());
    goalLayout.addNumber(
        "Arm Angle Goal (deg)", () -> Units.radiansToDegrees(goal.getAngleRadians()));
    goalLayout.addNumber("Arm Length Setpoint (m)", () -> trajectory.get().getLengthMeters());
    goalLayout.addNumber(
        "Arm Angle Setpoint (deg)",
        () -> Units.radiansToDegrees(trajectory.get().getAngleRadians()));
    goalLayout.addBoolean("At Goal?", this::atGoal);
    goalLayout.addBoolean("Is Enabled?", this::isEnabled);
  }

  @Override
  public void outputTelemetry() {}
}
