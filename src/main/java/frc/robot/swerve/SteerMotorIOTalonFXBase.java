package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.hardware.CAN;

/** Implements steer motor behaviors for a TalonFX. */
public abstract class SteerMotorIOTalonFXBase implements SteerMotorIO {

  /* The motor used for all strategies extending this class. */
  protected final TalonFX motor;

  /* The status signals used for getting position and velocity data. */
  protected final StatusSignal<Double> position, velocity;

  /* The CAN ID of the associated azimuth encoder, if applicable. */
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
    position.refresh();
    velocity.refresh();

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
