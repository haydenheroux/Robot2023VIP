// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  public double absoluteAngleRadians;

  public AzimuthEncoderIOSim(double absoluteAngleRadians) {
    this.absoluteAngleRadians = absoluteAngleRadians;
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.absoluteAngleRadians = absoluteAngleRadians;
  }
}
