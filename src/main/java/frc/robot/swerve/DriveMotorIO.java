package frc.robot.swerve;

/** Defines an interface for reading sensor data from and making requests to a drive motor. */
public interface DriveMotorIO {
  public static class DriveMotorIOValues {
    /** Position of the drive motor's encoder, in meters. */
    public double positionMeters = 0.0;
    /** Velocity of the drive motor's encoder, in meters per second. */
    public double velocityMetersPerSecond = 0.0;
  }

  /** Configures the drive motor. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(DriveMotorIOValues values);

  /**
   * Sets the drive motor's encoder to the distance, in meters.
   *
   * @param distanceMeters
   */
  public void setPosition(double distanceMeters);

  /**
   * Sets the drive motor's closed loop setpoint to the velocity, in meters per second.
   *
   * @param velocityMetersPerSecond
   */
  public void setVelocitySetpoint(double velocityMetersPerSecond);
}
