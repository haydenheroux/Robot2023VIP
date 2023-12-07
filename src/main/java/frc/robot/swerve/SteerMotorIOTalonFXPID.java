package frc.robot.swerve;

import frc.lib.hardware.CAN;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFXBase {

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPID(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }
}
