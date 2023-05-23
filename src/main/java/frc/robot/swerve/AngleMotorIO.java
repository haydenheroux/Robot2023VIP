package frc.robot.swerve;

public interface AngleMotorIO {
  public static class AngleMotorIOValues {
    /**
     * Position of the angle motor's encoder, in rotations. Wrapped within the range [0, 1)
     * rotations.
     */
    public double angleRotations = 0.0;
    /** Velocity of the angle motor's encoder, in rotations per second. */
    public double omegaRotationsPerSecond = 0.0;
  }

  /** Configures the angle motor hardware to default. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(AngleMotorIOValues values);

  /**
   * Sets the angle motor's encoder to the angle, in rotations.
   *
   * @param angleRotations
   */
  public void setPosition(double angleRotations);

  /**
   * Sets the angle motor's closed loop setpoint to the angle, in rotations.
   *
   * @param angleRotations
   */
  public void setSetpoint(double angleRotations);
}
