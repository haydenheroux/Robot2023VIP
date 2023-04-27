// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public interface SideIntakeIO {
  public static class SideIntakeIOValues {
    public double bottomMotorCurrentAmps = 0.0;
    public double topMotorCurrentAmps = 0.0;
  }

  public void configure();

  public void updateValues(SideIntakeIOValues values);

  public void setBottomMotorVoltage(double volts);

  public void setBottomMotorDisabled();

  public void setTopMotorVoltage(double volts);

  public void setTopMotorDisabled();
}
