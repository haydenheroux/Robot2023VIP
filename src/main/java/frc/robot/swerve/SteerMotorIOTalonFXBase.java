package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.CAN;
import frc.robot.Constants;

/** Implements asteer motor behaviors for a TalonFX. */
public abstract class SteerMotorIOTalonFXBase implements SteerMotorIO {

  protected final TalonFX motor;

  protected final StatusSignal<Double> position, velocity;

  protected final int encoderID;

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXBase(CAN motorCAN, CAN encoderCAN) {
    motor = new TalonFX(motorCAN.id, motorCAN.bus);

    position = motor.getPosition();
    velocity = motor.getVelocity();

    encoderID = encoderCAN.id;
  }

  @Override
  public abstract void configure();

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
  }

  @Override
  public abstract void setSetpoint(double angleRotations);
}
