package frc.robot.swerve;

public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  public AzimuthEncoderIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.angleRotations = 0;
  }
}
