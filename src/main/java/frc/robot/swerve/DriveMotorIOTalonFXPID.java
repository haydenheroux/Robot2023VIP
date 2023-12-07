package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.lib.math.Conversions;
import frc.robot.Constants.Physical;

/** Implements drive motor behaviors for a TalonFX. */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFXBase {

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

  private final PIDController feedback = new PIDController(0, 0, 0);

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
    double previousVelocityMetersPerSecond =
        Conversions.General.toMeters(velocity.getValue(), Physical.WHEEL_CIRCUMFERENCE);

    double feedforwardVolts = feedforward.calculate(velocityMetersPerSecond);
    double feedbackVolts = feedback.calculate(previousVelocityMetersPerSecond, velocityMetersPerSecond);

    double volts = feedforwardVolts + feedbackVolts;

    motor.setControl(new VoltageOut(volts));
  }
}
