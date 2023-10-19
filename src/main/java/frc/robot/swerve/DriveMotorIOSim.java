package frc.robot.swerve;

import frc.robot.Constants;

/**
 * Implements drive motor behaviors for a simulated drive motor.
 *
 * <p>Approximates the behavior of a physical drive motor.
 */
public class DriveMotorIOSim implements DriveMotorIO {

  private double positionMeters, velocityMetersPerSecond;

  public DriveMotorIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(DriveMotorIOValues values) {
    positionMeters += velocityMetersPerSecond * Constants.LOOP_TIME;

    values.positionMeters = positionMeters;
    values.velocityMetersPerSecond = velocityMetersPerSecond;
  }

  @Override
  public void setPosition(double distanceMeters) {
    positionMeters = distanceMeters;
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    this.velocityMetersPerSecond = velocityMetersPerSecond;
  }
}
