package frc.robot.swerve;

import frc.robot.Constants;

public class AngleMotorIOSim implements AngleMotorIO {

  private double angleRotations, omegaRotationsPerSecond;

  private final double kRotationsPerSecondPerVolt = 1 / 12.0;

  public AngleMotorIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(AngleMotorIOValues values) {
    double previousAngleRotations = angleRotations;
    angleRotations += omegaRotationsPerSecond * Constants.LOOP_TIME;

    double deltaAngleRotations = angleRotations - previousAngleRotations;

    double calculatedOmegaRadiansPerSecond = deltaAngleRotations / Constants.LOOP_TIME;

    values.angleRotations = angleRotations;
    values.omegaRotationsPerSecond = calculatedOmegaRadiansPerSecond;
  }

  @Override
  public void setPosition(double angleRotations) {
    this.angleRotations = angleRotations;
  }

  @Override
  public void setSetpoint(double angleRotations) {
    this.angleRotations = angleRotations;
  }

  @Override
  public void setVoltage(double volts) {
    omegaRotationsPerSecond = volts * kRotationsPerSecondPerVolt;
  }

  @Override
  public void setBrake(boolean isActive) {}
}
