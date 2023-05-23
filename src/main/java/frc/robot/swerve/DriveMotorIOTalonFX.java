package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Physical;
import frc.robot.Constants.Swerve.Drive;

public class DriveMotorIOTalonFX implements DriveMotorIO {

  private final TalonFX motor;
  private final SimpleMotorFeedforward feedforward;

  private final VelocityVoltage velocityController;

  public DriveMotorIOTalonFX(int id, String canbus) {
    motor = new TalonFX(id, canbus);

    double kv = Constants.NOMINAL_VOLTAGE / Constants.Swerve.MAX_SPEED;
    double ka = Constants.NOMINAL_VOLTAGE / Constants.Swerve.MAX_ACCELERATION;

    feedforward = new SimpleMotorFeedforward(0.0, kv, ka);

    velocityController = new VelocityVoltage(0);
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // TODO
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = Drive.KP;

    config.CurrentLimits.StatorCurrentLimit = Drive.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Drive.RAMP_TIME;

    config.Feedback.SensorToMechanismRatio = Drive.GEAR_RATIO;

    motor.getConfigurator().apply(config);

    motor.getPosition().setUpdateFrequency(100);
    motor.getVelocity().setUpdateFrequency(100);
  }

  @Override
  public void updateValues(DriveMotorIOValues values) {

    values.positionMeters =
        Conversions.General.toMeters(motor.getPosition().getValue(), Physical.WHEEL_CIRCUMFERENCE);
    values.velocityMetersPerSecond =
        Conversions.General.toMeters(motor.getVelocity().getValue(), Physical.WHEEL_CIRCUMFERENCE);
  }

  @Override
  public void setPosition(double distanceMeters) {
    double rotations =
        Conversions.General.toRotations(distanceMeters, Physical.WHEEL_CIRCUMFERENCE);

    motor.setRotorPosition(rotations);
  }

  @Override
  public void setSetpoint(double distanceMeters) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setSetpoint'");
  }

  @Override
  public void setVelocitySetpoint(double velocityMetersPerSecond) {
    double rotationsPerSecond =
        Conversions.General.toRotations(velocityMetersPerSecond, Physical.WHEEL_CIRCUMFERENCE);

    motor.setControl(
        velocityController
            .withVelocity(rotationsPerSecond)
            .withFeedForward(feedforward.calculate(velocityMetersPerSecond)));
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
