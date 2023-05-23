package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    // TODO
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = Angle.KP;
    config.Slot0.kD = Angle.KD;

    config.CurrentLimits.StatorCurrentLimit = Angle.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Angle.RAMP_TIME;

    config.Feedback.SensorToMechanismRatio = Angle.GEAR_RATIO;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateValues(AngleMotorIOValues values) {
    values.angleRotations = motor.getPosition().getValue();
    values.omegaRotationsPerSecond = motor.getVelocity().getValue();
  }

  @Override
  public void setPosition(double angleRotations) {
    motor.setRotorPosition(angleRotations);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motor.setControl(positionController.withPosition(angleRotations));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrake(boolean isActive) {
    // TODO
  }
}
