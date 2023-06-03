package frc.robot.swerve;

/** Defines an interface for reading sensor data from and making requests to a steer motor. */
public interface SteerMotorIO {
  public static class SteerMotorValues {
    /** Position of the steer motor's encoder, in rotations. */
    public double angleRotations = 0.0;
    /** Velocity of the steer motor's encoder, in rotations per second. */
    public double omegaRotationsPerSecond = 0.0;
  }

  /** Configures the steer motor hardware to default. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(SteerMotorValues values);

  /**
   * Sets the steer motor's encoder to the angle, in rotations.
   *
   * @param angleRotations
   */
  public void setPosition(double angleRotations);

  /**
   * Sets the steer motor's closed loop setpoint to the angle, in rotations.
   *
   * @param angleRotations
   */
  public void setSetpoint(double angleRotations);
}
