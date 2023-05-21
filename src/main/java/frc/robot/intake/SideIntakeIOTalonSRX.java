package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.Ports;

public class SideIntakeIOTalonSRX implements SideIntakeIO {

  private final WPI_TalonSRX bottomMotor, topMotor;

  public SideIntakeIOTalonSRX() {
    bottomMotor = new WPI_TalonSRX(Ports.BOTTOM_ROLLER);
    topMotor = new WPI_TalonSRX(Ports.TOP_ROLLER);
  }

  @Override
  public void configure() {
    bottomMotor.setInverted(true);
    topMotor.setInverted(true);
  }

  @Override
  public void updateValues(SideIntakeIOValues values) {
    values.bottomMotorCurrentAmps = Math.abs(bottomMotor.getStatorCurrent());
    values.topMotorCurrentAmps = Math.abs(topMotor.getStatorCurrent());
  }

  @Override
  public void setBottomMotorVoltage(double volts) {
    bottomMotor.setVoltage(volts);
  }

  @Override
  public void setBottomMotorDisabled() {
    bottomMotor.stopMotor();
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setTopMotorDisabled() {
    topMotor.stopMotor();
  }
}
