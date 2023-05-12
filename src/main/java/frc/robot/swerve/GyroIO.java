package frc.robot.swerve;

public interface GyroIO {
  public static class GyroIOValues {
    public double rollAngleRadians = 0.0;
    public double pitchAngleRadians = 0.0;
    public double yawAngleRadians = 0.0;

    public double xAccelerationMetersPerSecondSquared = 0.0;
    public double yAccelerationMetersPerSecondSquared = 0.0;
    public double zAccelerationMetersPerSecondSquared = 0.0;
  }

  public void configure();

  public void updateValues(GyroIOValues values);

  public void setRollAngle(double rollAngleRadians);

  public void setPitchAngle(double pitchAngleRadians);

  public void setYawAngle(double yawAngleRadians);
}
