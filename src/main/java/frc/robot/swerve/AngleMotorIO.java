package frc.robot.swerve;

public interface AngleMotorIO {
  public static class AngleMotorIOValues {
    public double angleRotations = 0.0;
    public double omegaRotationsPerSecond = 0.0;
  }

  public void configure();

  public void updateValues(AngleMotorIOValues values);

  public void setPosition(double angleRotations);

  public void setSetpoint(double angleRotations);

  public void setBrake(boolean isActive);
}
