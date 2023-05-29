package frc.robot.arm;

public interface ArmIO {
  public static class ArmIOValues {
    public boolean telescopingBrakeIsActive = true;
    public double telescopingLengthMeters = 0.0;
    public double telescopingVelocityMetersPerSecond = 0.0;
    public double telescopingVoltage = 0.0;

    public boolean pivotBrakeIsActive = true;
    public double pivotAngleRotations = 0.0;
    public double pivotOmegaRotationsPerSecond = 0.0;
    public double pivotVoltage = 0.0;
  }

  /** Configures arm hardware to default. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(ArmIOValues values);

  /**
   * Sets the telescoping motor position.
   *
   * @param lengthMeters
   */
  public void setTelescopingPosition(double lengthMeters);

  /**
   * Sets the telescoping motor setpoint.
   *
   * @param lengthMeters
   */
  public void setTelescopingSetpoint(double lengthMeters);

  /**
   * Sets the telescoping motor voltage.
   *
   * @param volts
   */
  public void setTelescopingVoltage(double volts);

  /**
   * Sets the telescoping brake.
   *
   * @param isActive
   */
  public void setTelescopingBrake(boolean isActive);

  /** Disables telescoping motor. */
  public void setTelescopingDisabled();

  /**
   * Sets the pivot motor position.
   *
   * @param angleRotations
   */
  public void setPivotPosition(double angleRotations);

  /**
   * Sets the pivot motor setpoint.
   *
   * @param angleRotations
   */
  public void setPivotSetpoint(double angleRotations);

  /**
   * Sets the pivot motor voltage.
   *
   * @param volts
   */
  public void setPivotVoltage(double volts);

  /**
   * Sets the pivot brake.
   *
   * @param isActive
   */
  public void setPivotBrake(boolean isActive);

  /** Disables pivot motor. */
  public void setPivotDisabled();
}
