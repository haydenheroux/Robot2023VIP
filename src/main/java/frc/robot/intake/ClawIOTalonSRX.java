package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class ClawIOTalonSRX implements ClawIO {

  private final WPI_TalonSRX motor;

  public ClawIOTalonSRX() {
    motor = new WPI_TalonSRX(RobotMap.CLAW);
  }

  @Override
  public void configure() {
    motor.setInverted(true);
  }

  @Override
  public void updateValues(ClawIOValues values) {
    values.motorCurrentAmps = Math.abs(motor.getStatorCurrent());
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setMotorDisabled() {
    motor.stopMotor();
  }
}
