// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;
import java.util.Objects;

public class ArmPosition {

  public final double extensionLengthMeters, rotationAngleRadians;

  public ArmPosition(double extensionLengthMeters, double rotationAngleRadians) {
    this.extensionLengthMeters = extensionLengthMeters;
    this.rotationAngleRadians = rotationAngleRadians;
  }

  public boolean approximatelyEquals(ArmPosition other) {
    return lengthEquals(other) && rotationEquals(other);
  }

  public boolean lengthEquals(ArmPosition other) {
    return Math.abs(this.extensionLengthMeters - other.extensionLengthMeters)
        < Constants.Arm.Extension.TOLERANCE;
  }

  public boolean rotationEquals(ArmPosition other) {
    return Math.abs(this.rotationAngleRadians - other.rotationAngleRadians)
        < Constants.Arm.Rotation.TOLERANCE;
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
