package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFXBase {

  private final double kS = 0.0;

  private final PIDController feedback = new PIDController(0, 0, 0);

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPID(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createSteerMotorConfig();

    motorConfig.Slot0 = new Slot0Configs();

    motorConfig.Feedback.FeedbackRemoteSensorID = 0;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    double previousAngleRotations = position.getValue();

    double feedbackVolts = feedback.calculate(previousAngleRotations, angleRotations);
    double feedforwardVolts = Math.signum(feedbackVolts) * kS;

    double volts = feedforwardVolts + feedbackVolts;

    motor.setControl(new VoltageOut(volts));
  }

}
