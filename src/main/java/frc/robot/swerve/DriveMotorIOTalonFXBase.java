package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.CAN;
import frc.lib.math.Conversions;
import frc.robot.Constants.Physical;

/** Implements drive motor behaviors for a TalonFX. */
public abstract class DriveMotorIOTalonFXBase implements DriveMotorIO {

  protected final TalonFX motor;

  protected final StatusSignal<Double> position, velocity;

  /**
   * Constructs a new TalonFX drive motor.
   *
   * @param can the CAN of the TalonFX.
   */
  public DriveMotorIOTalonFXBase(CAN can) {
    motor = new TalonFX(can.id, can.bus);

    position = motor.getPosition();
    velocity = motor.getVelocity();
  }

  @Override
  public abstract void configure();

  @Override
  public void updateValues(DriveMotorIOValues values) {
    values.positionMeters =
        Conversions.General.toMeters(
            BaseStatusSignal.getLatencyCompensatedValue(position, velocity),
            Physical.WHEEL_CIRCUMFERENCE);
    values.velocityMetersPerSecond =
        Conversions.General.toMeters(velocity.getValue(), Physical.WHEEL_CIRCUMFERENCE);
  }

  @Override
  public void setPosition(double distanceMeters) {
    double rotations =
        Conversions.General.toRotations(distanceMeters, Physical.WHEEL_CIRCUMFERENCE);

    motor.setPosition(rotations);
  }

  @Override
  public abstract void setVelocitySetpoint(double velocityMetersPerSecond);
}
