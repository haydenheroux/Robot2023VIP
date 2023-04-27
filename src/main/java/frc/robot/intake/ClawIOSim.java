// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public class ClawIOSim implements ClawIO {

    private double simMotorCurrent; 

    public boolean isHoldingCone, isHoldingCube;
    public double holdingConeCurrent, holdingCubeCurrent;

    private final double kResistance = 1.0;

    public ClawIOSim() {
    }

  @Override
  public void configure() {}

  @Override
  public void updateValues(ClawIOValues values) {
    values.motorCurrentAmps = simMotorCurrent;
  }

  @Override
  public void setMotorVoltage(double volts) {
    if (isHoldingCone) {
        simMotorCurrent = holdingConeCurrent;
    } else if (isHoldingCube) {
        simMotorCurrent = holdingCubeCurrent;
    } else {
        simMotorCurrent = volts * kResistance;
    }
  }

  @Override
  public void setMotorDisabled() {
    setMotorVoltage(0);
  }
}
