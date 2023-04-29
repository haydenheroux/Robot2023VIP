// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class SideIntakeIOTalonSRX implements SideIntakeIO {

  private final WPI_TalonSRX bottomMotor, topMotor;

  public SideIntakeIOTalonSRX() {
    bottomMotor = new WPI_TalonSRX(Constants.Intake.SideIntake.BOTTOM_CAN_ID);
    topMotor = new WPI_TalonSRX(Constants.Intake.SideIntake.TOP_CAN_ID);
  }

  @Override
  public void configure() {
    bottomMotor.setInverted(false);
    topMotor.setInverted(false);
  }

  @Override
  public void updateValues(SideIntakeIOValues values) {
    values.bottomMotorCurrentAmps = bottomMotor.getStatorCurrent();
    values.topMotorCurrentAmps = topMotor.getStatorCurrent();
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
