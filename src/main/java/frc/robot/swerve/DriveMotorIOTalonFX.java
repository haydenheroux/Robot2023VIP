package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Physical;
import frc.robot.Constants.Swerve;

/** Implements drive motor behaviors for a TalonFX. */
public class DriveMotorIOTalonFX implements DriveMotorIO {

  private final TalonFX motor;

  private final StatusSignal<Double> position, velocity;

  private final VelocityTorqueCurrentFOC velocityController;

  /**
   * Constructs a new TalonFX drive motor.
   *
   * @param can the CAN of the TalonFX.
   */
  public DriveMotorIOTalonFX(CAN can) {
    motor = new TalonFX(can.id, can.bus);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    velocityController = new VelocityTorqueCurrentFOC(0);
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Swerve.DRIVE_CONFIG, motor);
  }

  @Override
  public void updateValues(DriveMotorIOValues values) {
    position.waitForUpdate(Constants.LOOP_TIME);
    velocity.waitForUpdate(Constants.LOOP_TIME);

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
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    double rotationsPerSecond =
        Conversions.General.toRotations(velocityMetersPerSecond, Physical.WHEEL_CIRCUMFERENCE);

    motor.setControl(velocityController.withVelocity(rotationsPerSecond));
  }
}
