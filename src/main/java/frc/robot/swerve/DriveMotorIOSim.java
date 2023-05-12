package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class DriveMotorIOSim implements DriveMotorIO {

  private double positionMeters, velocityMetersPerSecond;

  private final PIDController positionController = new PIDController(0.0, 0.0, 0.0);

  private final double kMetersPerSecondPerVolt = Constants.Swerve.MAX_SPEED / 12.0;

  public DriveMotorIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(DriveMotorIOValues values) {
    double previousPositionMeters = positionMeters;
    positionMeters += velocityMetersPerSecond * Constants.LOOP_TIME;
    double deltaPositionMeters = positionMeters - previousPositionMeters;

    double calculatedVelocityMetersPerSecond = deltaPositionMeters / Constants.LOOP_TIME;

    values.positionMeters = positionMeters;
    values.velocityMetersPerSecond = calculatedVelocityMetersPerSecond;
  }

  @Override
  public void setPosition(double distanceMeters) {
    positionMeters = distanceMeters;
  }

  @Override
  public void setSetpoint(double distanceMeters) {
    velocityMetersPerSecond = 0.0;

    double volts = positionController.calculate(this.positionMeters, distanceMeters);
    setVoltage(volts);
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    this.velocityMetersPerSecond = velocityMetersPerSecond;
  }

  @Override
  public void setVoltage(double volts) {
    velocityMetersPerSecond = volts * kMetersPerSecondPerVolt;
  }

  @Override
  public void setBrake(boolean isActive) {}
}
