package frc.robot.intake;

public interface ClawIO {
  public static class ClawIOValues {
    public double motorCurrentAmps = 0.0;
  }

  public void configure();

  public void updateValues(ClawIOValues values);

  public void setMotorVoltage(double volts);

  public void setMotorDisabled();
}
