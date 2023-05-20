package frc.robot.swerve;

public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  public double absoluteAngleRotations;

  public AzimuthEncoderIOSim(double absoluteAngleRotations) {
    this.absoluteAngleRotations = absoluteAngleRotations;
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.absoluteAngleRotations = absoluteAngleRotations;
  }
}
