// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

public interface ArmIO {
  public static class ArmIOValues {
    public double extensionLengthMeters = 0.0;
    public boolean extensionBrakeIsActive = true;

    public double rotationAngleRadians = 0.0;
    public boolean rotationBrakeIsActive = true;
  }

  public void configure();

  public void updateValues(ArmIOValues values);

  public void setExtensionPosition(double lengthMeters);

  public void setExtensionSetpoint(double lengthMeters);

  public void setExtensionVoltage(double volts);

  public void setExtensionBrake(boolean isActive);

  public void setExtensionDisabled();

  public void setRotationPosition(double angleRadians);

  public void setRotationSetpoint(double angleRadians);

  public void setRotationVoltage(double volts);

  public void setRotationBrake(boolean isActive);

  public void setRotationDisabled();
}
