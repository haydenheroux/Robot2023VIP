package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.lib.math.Conversions;
import frc.robot.Constants.Physical;

/**
 * Implements drive motor behaviors for a voltage-controlled TalonFX using an external PID
 * controller.
 */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFXBase {

  /* Feedforward, in volts. */
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.139, 0);

  /* Velocity feedback controller. Outputs voltages to correct velocity error (in meters per second) */
  private final PIDController feedback = new PIDController(1, 0, 0);

  /**
   * Constructs a new TalonFX drive motor.
   *
   * @param can the CAN of the TalonFX.
   */
  public DriveMotorIOTalonFXPID(CAN can) {
    super(can);
  }

  @Override
  public void configure() {
    TalonFXConfiguration driveConfiguration = SwerveFactory.createDriveMotorConfig();

    ConfigurationApplier.apply(driveConfiguration, motor);
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    if (velocityMetersPerSecond == 0) {
      motor.setControl(new CoastOut());
    } else {
      motor.setControl(new VoltageOut(calculateVoltage(velocityMetersPerSecond)));
    }
  }

  private double calculateVoltage(double velocityMetersPerSecond) {
    /* Store the previous measurement of the process variable (velocity, in meters per second) for calculating feedback. */
    double previousVelocityMetersPerSecond =
        Conversions.General.toMeters(velocity.getValue(), Physical.WHEEL_CIRCUMFERENCE);

    /* Calculate feedback voltage to approach the requested velocity. */
    double feedbackVolts =
        feedback.calculate(previousVelocityMetersPerSecond, velocityMetersPerSecond);

    /* Calculate feedforward voltage to apply on top of feedback voltage. */
    double feedforwardVolts = feedforward.calculate(velocityMetersPerSecond);

    return feedbackVolts + feedforwardVolts;
  }
}
