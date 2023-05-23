package frc.robot.swerve;

import frc.robot.Constants;

public class DriveMotorIOSim implements DriveMotorIO {

  private double positionMeters, velocityMetersPerSecond;

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
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    this.velocityMetersPerSecond = velocityMetersPerSecond;
  }

  @Override
  public void setBrake(boolean isActive) {}
}
