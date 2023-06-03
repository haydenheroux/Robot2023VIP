package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/**
 * Implements steer motor behaviors for a simulated steer motor.
 *
 * <p>Approximates the behavior of a physical steer motor.
 */
public class SteerMotorIOSim implements SteerMotorIO {

  private double angleRotations, omegaRotationsPerSecond;

  private final double kOmegaRotationsPerSecondPerVolt = 1.0;

  private final PIDController angleController;

  /** Constructs a new simulated steer motor. */
  public SteerMotorIOSim() {
    angleController = new PIDController(Constants.Swerve.STEER_CONFIG.Slot0.kP, 0.0, 0.0);

    // From Phoenix 6 docs:
    // Wrap position error within [-0.5,+0.5) mechanism rotations. Typically used for continuous
    // position closed-loops like swerve azimuth.
    angleController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(SteerMotorValues values) {
    angleRotations += omegaRotationsPerSecond * Constants.LOOP_TIME;

    values.angleRotations = angleRotations;
    values.omegaRotationsPerSecond = omegaRotationsPerSecond;
  }

  @Override
  public void setPosition(double angleRotations) {
    this.angleRotations = angleRotations;
  }

  @Override
  public void setSetpoint(double angleRotations) {
    double volts = angleController.calculate(this.angleRotations, angleRotations);
    omegaRotationsPerSecond = volts * kOmegaRotationsPerSecondPerVolt;
  }
}
