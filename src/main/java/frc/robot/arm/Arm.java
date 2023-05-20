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
      double extensionLengthMeters = position.getLength() - Extension.LENGTH_OFFSET;
      return new State(extensionLengthMeters, position.getAngle());
    }
  }

  public enum Selector {
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

    setPosition(Positions.STOW);
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  /**
   * Sets the position of the arm to the specified position.
   *
   * @param position
   */
  public void setPosition(ArmPosition position) {
    this.position = position;

    State state = State.fromPosition(position);
    io.setExtensionPosition(state.extensionLengthMeters);
    io.setRotationPosition(state.rotationAngle.getRotations());
  }

  /**
   * Gets the position of the arm.
   *
   * @return
   */
  public ArmPosition getPosition() {
    return position;
  }

  /**
   * Tests whether the arm is intersecting the grid.
   *
   * @return true if the arm is intersecting the grid.
   */
  public boolean isIntersectingGrid() {
    return position.isIntersectingGrid();
  }
  /**
   * Tests whether the arm is within the rule zone.
   *
   * @return true if the arm is within the rule zone.
   */
  public boolean isWithinRuleZone() {
    return ArmKinematics.isWithinRuleZone(position);
  }

  /**
   * Disables parts of the arm.
   *
   * @param selector the part of the arm to be disabled.
   */
  public void disable(Selector selector) {
    switch (selector) {
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

  /**
   * Sets voltage for parts of the arm.
   *
   * @param selector the part of the arm to set voltage for.
   * @param volts the voltage to set the part of the arm to.
   */
  public void setVoltage(Selector selector, double volts) {
    switch (selector) {
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
   * Returns the brakes that are active.
   *
   * @return the brakes that are active.
   */
  public Selector getLocked() {
    if (values.extensionBrakeIsActive && values.rotationBrakeIsActive) return Selector.kBoth;
    if (values.extensionBrakeIsActive) return Selector.kExtension;
    if (values.rotationBrakeIsActive) return Selector.kRotation;
    return Selector.kNeither;
  }

  /**
   * Locks the selected brakes.
   *
   * @param selector the brakes to lock.
   */
  public void lock(Selector selector) {
    setBrake(selector, true);
  }

  /**
   * Unlocks the selected brakes.
   *
   * @param selector the brakes to unlock.
   */
  public void unlock(Selector selector) {
    setBrake(selector, false);
  }

  /**
   * Sets the selected brakes to be active or inactive.
   *
   * @param selector the brakes to set.
   * @param isActive whether the brakes are active or inactive.
   */
  private void setBrake(Selector selector, boolean isActive) {
    switch (selector) {
      case kBoth:
        io.setExtensionBrake(isActive);
        io.setRotationBrake(isActive);
        break;
      case kExtension:
        io.setExtensionBrake(isActive);
        break;
      case kNeither:
        break;
      case kRotation:
        io.setRotationBrake(isActive);
        break;
    }
  }

  /**
   * Tests if the extension of the arm is at the maximum.
   *
   * @return true if the extension of the arm is at the maximum.
   */
  public boolean extensionIsAtMax() {
    return values.extensionLengthMeters > Extension.MAX_LENGTH;
  }

  /**
   * Tests if the extension of the arm is at the minimum.
   *
   * @return true if the extension of the arm is at the minimum.
   */
  public boolean extensionIsAtMin() {
    return values.extensionLengthMeters < Extension.MIN_LENGTH;
  }

  /**
   * Tests if the rotation of the arm is at the maximum.
   *
   * @return true if the rotation of the arm is at the maximum.
   */
  public boolean rotationIsAtMax() {
    return values.rotationAngleRotations > Rotation.MAX_ANGLE.getRotations();
  }

  /**
   * Tests if the rotation of the arm is at the minimum.
   *
   * @return true if the rotation of the arm is at the minimum.
   */
  public boolean rotationIsAtMin() {
    return values.rotationAngleRotations < Rotation.MIN_ANGLE.getRotations();
  }

  /**
   * Tests if the arm is at the specified position.
   *
   * @param position the position to test.
   * @return true if the arm is at the specified position.
   */
  public boolean at(ArmPosition position) {
    return this.position.at(position);
  }

  /**
   * Sets the goal of the arm to the specified position.
   *
   * @param goal
   */
  public void setGoal(ArmPosition goal) {
    this.goal = goal;
  }

  /**
   * Sets the setpoint of the arm to the specified position.
   *
   * @param setpoint
   */
  public void setSetpoint(ArmPosition setpoint) {
    this.setpoint = setpoint;

    State state = State.fromPosition(setpoint);

    io.setExtensionSetpoint(state.extensionLengthMeters);
    io.setRotationSetpoint(state.rotationAngle.getRotations());
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    // Update the arm's position with IO readings
    position = ArmPosition.fromValues(values);

    // Update the arm's mechanism with new position
    SuperstructureMechanism.getInstance().updateArm(position, getLocked());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(getName());

    tab.addString(
            "Command",
            () -> this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "")
        .withPosition(0, 0)
        .withSize(1, 1);

    tab.addString("Is Locked?", () -> getLocked().toString()).withPosition(0, 1).withSize(1, 1);

    tab.add(this.runOnce(() -> this.unlock(Selector.kBoth)).withName("Unlock Both"))
        .withPosition(0, 2)
        .withSize(1, 1);

    tab.add(this.runOnce(() -> this.lock(Selector.kBoth)).withName("Lock Both"))
        .withPosition(0, 3)
        .withSize(1, 1);

    ShuffleboardLayout valuesLayout =
        tab.getLayout("Values", BuiltInLayouts.kList).withPosition(1, 0).withSize(2, 4);

    valuesLayout.addNumber("Extension Length (m)", () -> values.extensionLengthMeters);
    valuesLayout.addNumber(
        "Rotation Angle (deg)", () -> Units.rotationsToDegrees(values.rotationAngleRotations));
    valuesLayout.addBoolean("Extension Brake Is Active?", () -> values.extensionBrakeIsActive);
    valuesLayout.addBoolean("Rotation Brake Is Active?", () -> values.rotationBrakeIsActive);
    valuesLayout.addNumber("Extension Voltage (V)", () -> values.extensionVoltage);
    valuesLayout.addNumber("Rotation Voltage (V)", () -> values.rotationVoltage);

    ShuffleboardLayout positionLayout =
        tab.getLayout("Position", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 4);

    positionLayout.addNumber("Arm Length (m)", () -> position.getLength());
    positionLayout.addNumber("Arm Angle (deg)", () -> position.getAngle().getDegrees());

    ShuffleboardLayout setpointLayout =
        tab.getLayout("Setpoint", BuiltInLayouts.kList).withPosition(5, 0).withSize(2, 4);

    setpointLayout.addNumber("Arm Length Setpoint (m)", () -> setpoint.getLength());
    setpointLayout.addNumber("Arm Angle Setpoint (deg)", () -> setpoint.getAngle().getDegrees());
    setpointLayout.addBoolean("At Setpoint?", () -> at(setpoint));

    ShuffleboardLayout goalLayout =
        tab.getLayout("Goal", BuiltInLayouts.kList).withPosition(7, 0).withSize(2, 4);

    goalLayout.addNumber("Arm Length Goal (m)", () -> goal.getLength());
    goalLayout.addNumber("Arm Angle Goal (deg)", () -> goal.getAngle().getDegrees());
    goalLayout.addBoolean("At Goal?", () -> at(goal));
  }

  @Override
  public void outputTelemetry() {}

  public Command toGoal(ArmPosition goal) {
    return new ToGoal(this, goal);
  }

  public Command manualExtend(DoubleSupplier percentSupplier) {
    return new ManualExtend(this, percentSupplier);
  }

  public Command manualRotate(DoubleSupplier percentSupplier) {
    return new ManualRotate(this, percentSupplier);
  }
}
