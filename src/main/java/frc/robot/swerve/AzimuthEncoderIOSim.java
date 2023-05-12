package frc.robot.swerve;

public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  public double absoluteAngleRadians;

  public AzimuthEncoderIOSim(double absoluteAngleRadians) {
    this.absoluteAngleRadians = absoluteAngleRadians;
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.absoluteAngleRadians = absoluteAngleRadians;
  }
}
