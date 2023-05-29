package frc.robot.swerve;

/**
 * Implements azimuth encoder behaviors for a simulated azimuth encoder.
 *
 * <p>Approximates the behavior of a physical azimuth encoder.
 */
public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  public AzimuthEncoderIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.angleRotations = 0;
  }
}
