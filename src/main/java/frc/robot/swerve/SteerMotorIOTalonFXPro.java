package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXPro extends SteerMotorIOTalonFXIntegrated {

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPro(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createSteerMotorConfig();

    motorConfig.Slot0 = SwerveFactory.createSteerMotorGains();

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    ConfigurationApplier.apply(motorConfig, motor);
  }
}
