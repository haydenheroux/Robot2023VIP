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
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase implements TelemetryOutputter {
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

  private ArmPosition goal = new ArmPosition(0, 0);
  private ArmPosition setpoint = new ArmPosition(0, 0);

  private boolean reset = false;

  private ArmPosition position = new ArmPosition(0, 0);

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
    return position.approximatelyEquals(goal);
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
   * @param state the goal state.
   */
  public Command setGoal(ArmPosition state) {
    return this.runOnce(() -> this.goal = state);
  }

  /**
   * Returns if the arm position has been reset.
   *
   * @return if the arm position has been reset.
   */
  public boolean isReset() {
    return reset;
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
   * Resets the position of the arm to the state.
   *
   * @param state the state.
   */
  public void reset(ArmPosition state) {
    reset = true;
    io.setExtensionPosition(state.extensionLengthMeters);
    io.setRotationPosition(state.rotationAngleRadians);
  }

  /**
   * Returns the current position of the arm.
   *
   * @return the current position of the arm.
   */
  public ArmPosition getState() {
    return new ArmPosition(values.rotationAngleRadians, values.extensionLengthMeters);
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

          boolean isWithinRuleZone = ArmConstraintsSolver.isWithinRuleZone(position);
          boolean isLeavingRuleZone = !isWithinRuleZone && extensionIncreasing;

          if (!extensionPastMin && !extensionPastMax && !isLeavingRuleZone) {
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
    setpoint = ArmTrajectory.next(position, goal);
    io.setExtensionSetpoint(setpoint.extensionLengthMeters);
    io.setRotationSetpoint(setpoint.rotationAngleRadians);
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    position = new ArmPosition(values.extensionLengthMeters, values.rotationAngleRadians);

    if (isEnabled()) updateSetpoint();

    SuperstructureMechanism.getInstance().updateArm(position, getLocked());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString("Is Locked?", () -> getLocked().toString());

    ShuffleboardLayout valuesLayout = tab.getLayout("Values", BuiltInLayouts.kList);
    valuesLayout.addNumber("Extension Length (m)", () -> values.extensionLengthMeters);
    valuesLayout.addNumber("Rotation Angle (deg)", () -> Units.radiansToDegrees(values.rotationAngleRadians));
    valuesLayout.addBoolean("Extension Brake Is Active?", () -> values.extensionBrakeIsActive);
    valuesLayout.addBoolean("Rotation Brake Is Active?", () -> values.rotationBrakeIsActive);

    ShuffleboardLayout goalLayout = tab.getLayout("Goal", BuiltInLayouts.kList);
    goalLayout.addNumber("Extension Length Goal (m)", () -> goal.extensionLengthMeters);
    goalLayout.addNumber("Rotation Angle Goal (deg)", () -> Units.radiansToDegrees(goal.rotationAngleRadians));
    goalLayout.addNumber("Extension Length Setpoint (m)", () -> setpoint.extensionLengthMeters);
    goalLayout.addNumber("Rotation Angle Setpoint (deg)", () -> Units.radiansToDegrees(setpoint.rotationAngleRadians));
    goalLayout.addBoolean("At Goal?", this::atGoal);
    goalLayout.addBoolean("Is Enabled?", this::isEnabled);
  }

  @Override
  public void outputTelemetry() {}
}
