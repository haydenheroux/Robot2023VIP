package frc.robot.swerve;

import frc.robot.Constants;

public class AngleMotorIOSim implements AngleMotorIO {

  private double angleRadians, omegaRadiansPerSecond;

  private final double kRadiansPerSecondPerVolt = 2.0 * Math.PI / 12.0;

  public AngleMotorIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(AngleMotorIOValues values) {
    double previousAngleRadians = angleRadians;
    angleRadians += omegaRadiansPerSecond * Constants.LOOP_TIME;

    double deltaAngleRadians = angleRadians - previousAngleRadians;

    double calculatedOmegaRadiansPerSecond = deltaAngleRadians / Constants.LOOP_TIME;

    values.angleRadians = angleRadians;
    values.omegaRadiansPerSecond = calculatedOmegaRadiansPerSecond;
  }

  @Override
  public void setPosition(double angleRadians) {
    this.angleRadians = angleRadians;
  }

  @Override
  public void setSetpoint(double angleRadians) {
    this.angleRadians = angleRadians;
  }

  @Override
  public void setVoltage(double volts) {
    omegaRadiansPerSecond = volts * kRadiansPerSecondPerVolt;
  }

  @Override
  public void setBrake(boolean isActive) {}
}
