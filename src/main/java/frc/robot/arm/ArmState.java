// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;
import java.util.Objects;

public class ArmState {

  public final double extensionLengthMeters, rotationAngleRadians;

  public ArmState(double extensionLengthMeters, double rotationAngleRadians) {
    this.extensionLengthMeters = extensionLengthMeters;
    this.rotationAngleRadians = rotationAngleRadians;
  }

  public static ArmState fromPosition(ArmPosition position) {
    double extensionLengthMeters = position.getNorm() - Constants.Arm.Extension.LENGTH_OFFSET;
    double rotationAngleRadians = position.getAngle().getRadians();
    return new ArmState(extensionLengthMeters, rotationAngleRadians);
  }

  public boolean at(ArmState other) {
    return atLengthOf(other) && atRotationOf(other);
  }

  public boolean atLengthOf(ArmState other) {
    return Math.abs(this.extensionLengthMeters - other.extensionLengthMeters)
        < Constants.Arm.Extension.TOLERANCE;
  }

  public boolean atRotationOf(ArmState other) {
    return Math.abs(this.rotationAngleRadians - other.rotationAngleRadians)
        < Constants.Arm.Rotation.TOLERANCE;
  }

  @Override
  public boolean equals(Object other) {
    if (other instanceof ArmState) {
      ArmState rhs = (ArmState) other;
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
