package frc.robot.swerve;

/** Defines an interface for reading sensor data from an azimuth encoder. */
public interface AzimuthEncoderIO {
  public static class AzimuthEncoderIOValues {
    /** Position measured by the azimuth encoder, in rotations. */
    public double angleRotations = 0.0;
  }

  /** Configures azimuth encoder to a standard configuration. */
  public void configure();

  /**
   * Updates values with sensor information.
   *
   * @param values
   */
  public void updateValues(AzimuthEncoderIOValues values);
}
