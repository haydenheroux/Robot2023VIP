package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

/** Implements angle motor behaviors for a TalonFX. */
public class AngleMotorIOTalonFX implements AngleMotorIO {

  private final TalonFX motor;

  private final StatusSignal<Double> position, velocity;

  private final PositionVoltage positionController;

  /**
   * Constructs a new TalonFX angle motor.
   *
   * @param id the CAN ID of the TalonFX.
   * @param canbus the name of the CAN bus for the TalonFX.
   */
  public AngleMotorIOTalonFX(int id, String canbus) {
    motor = new TalonFX(id, canbus);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    positionController = new PositionVoltage(0);
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Swerve.ANGLE_CONFIG, motor);

    position.setUpdateFrequency(100);
    velocity.setUpdateFrequency(100);
  }

  @Override
  public void updateValues(AngleMotorIOValues values) {
    if (true) {
      position.refresh();
      velocity.refresh();
    }

    values.angleRotations = BaseStatusSignal.getLatencyCompensatedValue(position, velocity);
    values.omegaRotationsPerSecond = velocity.getValue();
  }

  @Override
  public void setPosition(double angleRotations) {
    motor.setRotorPosition(angleRotations);
    position.waitForUpdate(0.1);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motor.setControl(positionController.withPosition(angleRotations));
  }
}
