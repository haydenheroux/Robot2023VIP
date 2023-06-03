package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.Mechanisms;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase implements TelemetryOutputter {
  public enum Selector {
    kBoth,
    kTelescoping,
    kNeither,
    kPivot,
  }

  // Singleton instance
  private static Arm instance = null;

  // ArmIO handler
  private final ArmIO io;
  private final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  private ArmPosition position = ArmPosition.STOW;
  private ArmPosition goal = position;
  private ArmPosition setpoint = goal;

  /** Creates a new Arm. */
  private Arm() {
    if (Robot.isSimulation()) {
      io = new ArmIOSim();
    } else {
      io = new ArmIOSim();
    }

    io.configure();

    setPosition(ArmPosition.STOW);
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

    io.setTelescopingPosition(position.getExtension());
    io.setPivotPosition(position.getAngle().getRotations());
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
    return ArmConstraints.isWithinRuleZone(position);
  }

  /**
   * Disables parts of the arm.
   *
   * @param selector the part of the arm to be disabled.
   */
  public void disable(Selector selector) {
    switch (selector) {
      case kBoth:
        io.setTelescopingDisabled();
        io.setPivotDisabled();
        break;
      case kTelescoping:
        io.setTelescopingDisabled();
        break;
      case kNeither:
        break;
      case kPivot:
        io.setPivotDisabled();
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
        io.setTelescopingVoltage(volts);
        io.setPivotVoltage(volts);
        break;
      case kTelescoping:
        io.setTelescopingVoltage(volts);
        break;
      case kNeither:
        break;
      case kPivot:
        io.setPivotVoltage(volts);
        break;
    }
  }

  /**
   * Returns the brakes that are active.
   *
   * @return the brakes that are active.
   */
  public Selector getLocked() {
    if (values.telescopingBrakeIsActive && values.pivotBrakeIsActive) return Selector.kBoth;
    if (values.telescopingBrakeIsActive) return Selector.kTelescoping;
    if (values.pivotBrakeIsActive) return Selector.kPivot;
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
        io.setTelescopingBrake(isActive);
        io.setPivotBrake(isActive);
        break;
      case kTelescoping:
        io.setTelescopingBrake(isActive);
        break;
      case kNeither:
        break;
      case kPivot:
        io.setPivotBrake(isActive);
        break;
    }
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

    io.setTelescopingSetpoint(setpoint.getExtension());
    io.setPivotSetpoint(setpoint.getAngle().getRotations());
  }

  @Override
  public void periodic() {
    io.updateValues(values);

    // Update the arm's position with IO readings
    position = ArmPosition.fromValues(values);

    // Update the arm's mechanism with new position
    Mechanisms.getInstance().updateArm(position, getLocked());
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

    valuesLayout.addNumber("Telescoping Length (m)", () -> values.telescopingLengthMeters);
    valuesLayout.addNumber(
        "Telescoping Velocity (mps)", () -> values.telescopingVelocityMetersPerSecond);
    valuesLayout.addNumber(
        "Pivot Angle (deg)", () -> Units.rotationsToDegrees(values.pivotAngleRotations));
    valuesLayout.addNumber(
        "Pivot Velocity (dps)",
        () -> Units.rotationsToDegrees(values.pivotOmegaRotationsPerSecond));
    valuesLayout.addBoolean("Telescoping Brake Is Active?", () -> values.telescopingBrakeIsActive);
    valuesLayout.addBoolean("Pivot Brake Is Active?", () -> values.pivotBrakeIsActive);
    valuesLayout.addNumber("Telescoping Voltage (V)", () -> values.telescopingVoltage);
    valuesLayout.addNumber("Pivot Voltage (V)", () -> values.pivotVoltage);

    ShuffleboardLayout positionLayout =
        tab.getLayout("Position", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 4);

    positionLayout.addNumber("Arm Length (m)", () -> position.getLength());
    positionLayout.addNumber("Arm Angle (deg)", () -> position.getAngle().getDegrees());

    ShuffleboardLayout setpointLayout =
        tab.getLayout("Setpoint", BuiltInLayouts.kList).withPosition(5, 0).withSize(2, 4);

    setpointLayout.addNumber("Arm Length Setpoint (m)", () -> setpoint.getLength());
    setpointLayout.addNumber("Arm Angle Setpoint (deg)", () -> setpoint.getAngle().getDegrees());
    setpointLayout.addBoolean("At Setpoint?", () -> position.at(setpoint));

    ShuffleboardLayout goalLayout =
        tab.getLayout("Goal", BuiltInLayouts.kList).withPosition(7, 0).withSize(2, 4);

    goalLayout.addNumber("Arm Length Goal (m)", () -> goal.getLength());
    goalLayout.addNumber("Arm Angle Goal (deg)", () -> goal.getAngle().getDegrees());
    goalLayout.addBoolean("At Goal?", () -> position.at(goal));
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

  public Command characterize(Selector selector, String key) {
    return new Characterize(this, selector, key);
  }
}
