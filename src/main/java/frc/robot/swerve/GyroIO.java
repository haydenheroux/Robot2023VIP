package frc.robot.swerve;

public interface GyroIO {
  public static class GyroIOValues {
    public double rollAngleRotations = 0.0;
    public double pitchAngleRotations = 0.0;
    public double yawAngleRotations = 0.0;

    public double rollAccelerationG = 0.0;
    public double pitchAccelerationG = 0.0;
    public double yawAccelerationG = 0.0;
  }

  public void configure();

  public void updateValues(GyroIOValues values);

  public void setYawAngle(double yawAngleRotations);
}
