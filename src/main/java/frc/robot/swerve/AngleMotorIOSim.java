package frc.robot.swerve;

import frc.robot.Constants;

/**
 * Implements angle motor behaviors for a simulated angle motor.
 *
 * <p>Approximates the behavior of a physical angle motor.
 */
public class AngleMotorIOSim implements AngleMotorIO {

  private double angleRotations, omegaRotationsPerSecond;

  /** Constructs a new simulated angle motor. */
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
}
