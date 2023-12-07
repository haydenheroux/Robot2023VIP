package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.lib.math.Conversions;
import frc.robot.Constants.Physical;

/** Implements drive motor behaviors for a TalonFX. */
public class DriveMotorIOTalonFXIntegrated extends DriveMotorIOTalonFXBase {

  /**
   * Constructs a new TalonFX drive motor.
   *
   * @param can the CAN of the TalonFX.
   */
  public DriveMotorIOTalonFXIntegrated(CAN can) {
    super(can);
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createDriveMotorConfig();

    motorConfig.Slot0 = SwerveFactory.createDriveMotorGains();

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    double rotationsPerSecond =
        Conversions.General.toRotations(velocityMetersPerSecond, Physical.WHEEL_CIRCUMFERENCE);

    motor.setControl(new VelocityVoltage(rotationsPerSecond));
  }
}
