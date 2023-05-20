package frc.robot.swerve;

public interface GyroIO {
  public static class GyroIOValues {
    public double rollAngleRotations = 0.0;
    public double pitchAngleRotations = 0.0;
    public double yawAngleRotations = 0.0;

    public double xAccelerationMetersPerSecondSquared = 0.0;
    public double yAccelerationMetersPerSecondSquared = 0.0;
    public double zAccelerationMetersPerSecondSquared = 0.0;
  }

  public void configure();

  public void updateValues(GyroIOValues values);

  public void setRollAngle(double rollAngleRotations);

  public void setPitchAngle(double pitchAngleRotations);

  public void setYawAngle(double yawAngleRotations);
}
