package frc.robot.intake;

public interface SideIntakeIO {
  public static class SideIntakeIOValues {
    public double bottomMotorCurrentAmps = 0.0;
    public double topMotorCurrentAmps = 0.0;
  }

  public void configure();

  public void updateValues(SideIntakeIOValues values);

  public void setBottomMotorVoltage(double volts);

  public void setBottomMotorDisabled();

  public void setTopMotorVoltage(double volts);

  public void setTopMotorDisabled();
}
