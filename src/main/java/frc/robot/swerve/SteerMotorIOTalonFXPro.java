package frc.robot.swerve;

import frc.lib.hardware.CAN;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXPro extends SteerMotorIOTalonFX {

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXPro(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }
}
