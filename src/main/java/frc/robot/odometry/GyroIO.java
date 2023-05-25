package frc.robot.odometry;

public interface GyroIO {
  public static class GyroIOValues {
    /** Angle of the roll axis, in rotations. */
    public double rollAngleRotations = 0.0;
    /** Angle of the pitch axis, in rotations. */
    public double pitchAngleRotations = 0.0;
    /** Angle of the yaw axis, in rotations. */
    public double yawAngleRotations = 0.0;

    /** Acceleration of the roll axis, in Gs. */
    public double rollAccelerationG = 0.0;
    /** Acceleration of the pitch axis, in Gs. */
    public double pitchAccelerationG = 0.0;
    /** Acceleration of the yaw axis, in Gs. */
    public double yawAccelerationG = 0.0;
  }

  /** Configures the gyroscope hardware to default. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(GyroIOValues values);

  /**
   * Sets the angle of the gyroscope's yaw, in rotations.
   *
   * @param yawAngleRotations
   */
  public void setYawAngle(double yawAngleRotations);
}
