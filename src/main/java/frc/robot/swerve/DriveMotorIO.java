package frc.robot.swerve;

public interface DriveMotorIO {
  public static class DriveMotorIOValues {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
  }

  public void configure();

  public void updateValues(DriveMotorIOValues values);

  public void setPosition(double distanceMeters);

  public void setVelocitySetpoint(double velocityMetersPerSecond);

  public void setBrake(boolean isActive);
}
