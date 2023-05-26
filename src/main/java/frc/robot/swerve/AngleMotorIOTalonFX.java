package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

public class AngleMotorIOTalonFX implements AngleMotorIO {

  private final TalonFX motor;

  private final StatusSignal<Double> position, velocity;

  private final PositionVoltage positionController;

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
    // https://github.com/TitaniumTitans/2023ChargedUp/blob/0306f0274d170ba5cd87808f60e1d64475917b67/src/main/java/frc/robot/subsystems/swerve/module/FalconProModule.java#L136
    values.angleRotations = normalize(position.getValue());
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

  /**
   * Wraps a number of rotations to an absolute value in the range [0, 1) rotations.
   *
   * @param rotations
   * @return rotations, [0, 1).
   */
  private double normalize(double rotations) {
    if (rotations < 0) {
      rotations = 1 - (-rotations % 1);
    }

    return rotations % 1;
  }
}
