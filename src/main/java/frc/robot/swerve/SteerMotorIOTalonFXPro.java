package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

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

    /* For this strategy, a remote azimuth encoder is used to periodically update the position of integrated controller. */
    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    /* Since the feedback sensor is at the wheel, the gear ratio will be from the motor to the sensor. */
    motorConfig.Feedback.RotorToSensorRatio = Swerve.MK4I.STEER_RATIO;

    ConfigurationApplier.apply(motorConfig, motor);
  }
}
