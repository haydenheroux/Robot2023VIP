package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.lib.hardware.ConfigurationApplier;
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
   * @param motorID the CAN ID of the TalonFX.
   * @param encoderID the CAN ID of the CANcoder.
   * @param canbus the name of the CAN bus for the TalonFX.
   */
  public SteerMotorIOTalonFX(int motorID, int encoderID, String canbus) {
    motor = new TalonFX(motorID, canbus);
    this.encoderID = encoderID;

    position = motor.getPosition();
    velocity = motor.getVelocity();

    positionController = new PositionVoltage(0);
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = Swerve.STEER_CONFIG;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    ConfigurationApplier.apply(motorConfig, motor);

    position.setUpdateFrequency(100);
    velocity.setUpdateFrequency(100);
  }

  @Override
  public void updateValues(SteerMotorValues values) {
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
