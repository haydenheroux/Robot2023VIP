package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

/**
 * Implements steer motor behaviors for a voltage-controlled TalonFX using an external PID
 * controller.
 */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFXBase {

  /* Static feedforward, in volts. This constant approximately equals the voltage required to overcome static forces in the motor, such as friction, resulting in barely detectable motion at the wheel. */
  private final double kS = 0.14;

  /* Position feedback controller. Outputs voltages to correct positional error (in rotations) of this steer motor. */
  private final PIDController feedback = new PIDController(4, 0, 0);

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPID(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);

    /* Allow wrapping full rotations. */
    feedback.enableContinuousInput(0, 1);

    // TODO Determine acceptable tolerance
    feedback.setTolerance(Units.degreesToRotations(3));
  }

  /**
   * Configures the steer motor.
   *
   * <p>Since this strategy does not use integrated features of the TalonFX, few alterations to the
   * base configuration are made.
   */
  @Override
  public void configure() {
    TalonFXConfiguration motorConfig = SwerveFactory.createSteerMotorConfig();

    /* Since the feedback sensor is at the motor, the gear ratio will be from the motor to the wheel. */
    motorConfig.Feedback.SensorToMechanismRatio = Swerve.MK4I.STEER_RATIO;

    ConfigurationApplier.apply(motorConfig, motor);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    double feedbackVolts = feedback.calculate(position.getValue(), angleRotations);

    /* Apply static feedforward voltage on top of feedback voltage. */
    double feedforwardVolts = Math.signum(feedbackVolts) * kS;

    if (feedback.atSetpoint()) {
      motor.setControl(new CoastOut());
    } else {
      motor.setControl(new VoltageOut(feedbackVolts + feedforwardVolts));
    }
  }
}
