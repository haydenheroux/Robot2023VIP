package frc.robot.swerve;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.lib.hardware.CAN;
import frc.lib.math.Conversions;
import frc.robot.Constants.Physical;

/** Implements drive motor behaviors for a TalonFX. */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFXBase {

  /**
   * Constructs a new TalonFX drive motor.
   *
   * @param can the CAN of the TalonFX.
   */
  public DriveMotorIOTalonFXPID(CAN can) {
    super(can);
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    double rotationsPerSecond =
        Conversions.General.toRotations(velocityMetersPerSecond, Physical.WHEEL_CIRCUMFERENCE);

    motor.setControl(new VelocityVoltage(rotationsPerSecond));
  }
}
