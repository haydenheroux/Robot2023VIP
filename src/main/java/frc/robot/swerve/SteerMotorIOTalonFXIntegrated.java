package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXIntegrated extends SteerMotorIOTalonFXBase {

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXIntegrated(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createSteerMotorConfig();

    motorConfig.Slot0 = SwerveFactory.createSteerMotorGains();

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.RotorToSensorRatio = Swerve.MK4I.STEER_RATIO;

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motor.setControl(new PositionVoltage(angleRotations));
  }
}
