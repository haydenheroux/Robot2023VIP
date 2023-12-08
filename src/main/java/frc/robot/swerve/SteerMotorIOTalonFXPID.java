package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFXBase {

  private final double kS = 0.14;

  private final PIDController feedback = new PIDController(4, 0, 0);

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPID(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);

    feedback.enableContinuousInput(0, 1);
    feedback.setTolerance(Units.degreesToRotations(3));
  }

  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createSteerMotorConfig();

    motorConfig.Feedback.SensorToMechanismRatio = Swerve.MK4I.STEER_RATIO;

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    double previousAngleRotations = position.getValue();

    double feedbackVolts = feedback.calculate(previousAngleRotations, angleRotations);
    double feedforwardVolts = Math.signum(feedbackVolts) * kS;

    double volts = feedforwardVolts + feedbackVolts;

    if (feedback.atSetpoint()) volts = 0;

    motor.setControl(new VoltageOut(volts));
  }
}
