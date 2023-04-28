// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import java.util.Objects;

public class ArmPosition {

  public double extensionLengthMeters, rotationAngleRadians;

  public ArmPosition() {}

  public ArmPosition(double extensionLengthMeters, double rotationAngleRadians) {
    this.extensionLengthMeters = extensionLengthMeters;
    this.rotationAngleRadians = rotationAngleRadians;
  }

  @Override
  public boolean equals(Object other) {
    if (other instanceof ArmPosition) {
      ArmPosition rhs = (ArmPosition) other;
      boolean extensionLengthsEqual = this.extensionLengthMeters == rhs.extensionLengthMeters;
      boolean rotationAnglesEqual = this.rotationAngleRadians == rhs.rotationAngleRadians;
      return extensionLengthsEqual && rotationAnglesEqual;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(extensionLengthMeters, rotationAngleRadians);
  }
}
