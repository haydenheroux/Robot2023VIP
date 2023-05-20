package frc.robot.swerve;

public interface AzimuthEncoderIO {
  public static class AzimuthEncoderIOValues {
    public double absoluteAngleRotations = 0.0;
  }

  public void configure();

  public void updateValues(AzimuthEncoderIOValues values);
}
