package frc.robot.swerve;

import frc.lib.hardware.CAN;

/** Implements asteer motor behaviors for a TalonFX. */
public class SteerMotorIOTalonFXIntegrated extends SteerMotorIOTalonFXBase {

  /**
   * Constructs a new TalonFX steer motor.
   *
   * @param motorCAN the CAN of the TalonFX.
   * @param encoderCAN the CAN of the CANcoder.
   */
  public SteerMotorIOTalonFXIntegrated(CAN motorCAN, CAN encoderCAN) {
    super(motorCAN, encoderCAN);
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motor.setControl(positionController.withPosition(angleRotations));
  }
}
