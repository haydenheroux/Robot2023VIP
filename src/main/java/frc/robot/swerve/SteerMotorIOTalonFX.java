package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFX implements SteerMotorIO {

  private final TalonFX motor;

  private final StatusSignal<Double> position, velocity;

  private final PositionVoltage positionController;

  private final int encoderID;

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFX(CAN motorCAN, CAN encoderCAN) {
    motor = new TalonFX(motorCAN.id, motorCAN.bus);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    positionController = new PositionVoltage(0);

    encoderID = encoderCAN.id;
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = Swerve.STEER_CONFIG;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;

    if (Constants.USE_PRO) {
      motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    } else {
      motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    }

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void updateValues(SteerMotorValues values) {
    position.waitForUpdate(Constants.LOOP_TIME);
    velocity.waitForUpdate(Constants.LOOP_TIME);

    values.angleRotations = BaseStatusSignal.getLatencyCompensatedValue(position, velocity);
    values.omegaRotationsPerSecond = velocity.getValue();
  }

  @Override
  public void setPosition(double angleRotations) {
    motor.setPosition(angleRotations);
    position.waitForUpdate(0.1);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motor.setControl(positionController.withPosition(angleRotations));
  }
}
