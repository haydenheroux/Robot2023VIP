package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.Swerve.Angle;

public class AngleMotorIOTalonFX implements AngleMotorIO {

  private final TalonFX motor;

  private final PositionVoltage positionController;

  public AngleMotorIOTalonFX(int id, String canbus) {
    motor = new TalonFX(id, canbus);

    positionController = new PositionVoltage(0);
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = Angle.KP;
    config.Slot0.kD = Angle.KD;

    config.CurrentLimits.StatorCurrentLimit = Angle.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Angle.RAMP_TIME;

    config.Feedback.SensorToMechanismRatio = Angle.GEAR_RATIO;

    /*
     * https://github.com/TitaniumTitans/2023ChargedUp/blob/0306f0274d170ba5cd87808f60e1d64475917b67/src/main/java/frc/robot/subsystems/swerve/module/FalconProModule.java#L201
     */
    StatusCode status;
    do {
      status = motor.getConfigurator().apply(config);
    } while (!status.isOK());

    motor.getPosition().setUpdateFrequency(100);
    motor.getVelocity().setUpdateFrequency(100);
  }

  @Override
  public void updateValues(AngleMotorIOValues values) {
    /*
     * https://github.com/TitaniumTitans/2023ChargedUp/blob/0306f0274d170ba5cd87808f60e1d64475917b67/src/main/java/frc/robot/subsystems/swerve/module/FalconProModule.java#L136
     */
    values.angleRotations = normalize(motor.getPosition().getValue());
    values.omegaRotationsPerSecond = motor.getVelocity().getValue();
  }

  @Override
  public void setPosition(double angleRotations) {
    motor.setRotorPosition(angleRotations);
    motor.getPosition().waitForUpdate(0.1);
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
