package frc.robot.arm;

public interface ArmIO {
  public static class ArmIOValues {
    public boolean extensionBrakeIsActive = true;
    public double extensionLengthMeters = 0.0;
    public double extensionVoltage = 0.0;

    public boolean rotationBrakeIsActive = true;
    public double rotationAngleRotations = 0.0;
    public double rotationVoltage = 0.0;
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
   * Sets the extension motor position.
   *
   * @param lengthMeters
   */
  public void setExtensionPosition(double lengthMeters);

  /**
   * Sets the extension motor setpoint.
   *
   * @param lengthMeters
   */
  public void setExtensionSetpoint(double lengthMeters);

  /**
   * Sets the extension motor voltage.
   *
   * @param volts
   */
  public void setExtensionVoltage(double volts);

  /**
   * Sets the extension brake.
   *
   * @param isActive
   */
  public void setExtensionBrake(boolean isActive);

  /** Disables extension motor. */
  public void setExtensionDisabled();

  /**
   * Sets the rotation motor position.
   *
   * @param angleRotations
   */
  public void setRotationPosition(double angleRotations);

  /**
   * Sets the rotation motor setpoint.
   *
   * @param angleRotations
   */
  public void setRotationSetpoint(double angleRotations);

  /**
   * Sets the rotation motor voltage.
   *
   * @param volts
   */
  public void setRotationVoltage(double volts);

  /**
   * Sets the rotation brake.
   *
   * @param isActive
   */
  public void setRotationBrake(boolean isActive);

  /** Disables rotation motor. */
  public void setRotationDisabled();
}
