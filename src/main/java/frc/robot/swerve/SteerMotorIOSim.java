package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.MK4I;

/**
 * Implements steer motor behaviors for a simulated steer motor.
 *
 * <p>Approximates the behavior of a physical steer motor.
 */
public class SteerMotorIOSim implements SteerMotorIO {

  private double angleRotations, omegaRotationsPerSecond;

  private final double kMaxRotationsPerSecondAt12Volts = ((6380.0 / 60.0) / MK4I.STEER_RATIO);
  private final double kRotationsPerSecondPerVolt = kMaxRotationsPerSecondAt12Volts / 12.0;

  private final PIDController angleController;

  /** Constructs a new simulated steer motor. */
  public SteerMotorIOSim() {
    angleController = new PIDController(Constants.Swerve.STEER_KP.getSimulated(), 0.0, 0.0);

    // From Phoenix 6 docs:
    // Default azimuth encoder strategy is AbsoluteSensorRangeValue.Unsigned_0To1
    angleController.enableContinuousInput(0, 1);
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
    omegaRotationsPerSecond = volts * kRotationsPerSecondPerVolt;
  }
}
