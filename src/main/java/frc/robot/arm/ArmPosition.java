// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class ArmPosition extends Translation2d {

  public ArmPosition(double lengthMeters, double angleRadians) {
    super(lengthMeters, Rotation2d.fromRadians(angleRadians));
  }

  public static ArmPosition fromState(ArmState position) {
    double length = position.extensionLengthMeters + Constants.Arm.Extension.LENGTH_OFFSET;
    return new ArmPosition(length, position.rotationAngleRadians);
  }

  public double getAngleRadians() {
    return super.getAngle().getRadians();
  }

  public double getLengthMeters() {
    return super.getNorm();
  }

  public boolean at(ArmPosition other) {
    return atLengthOf(other) && atAngleOf(other);
  }

  public boolean atLengthOf(ArmPosition other) {
    return Math.abs(this.getLengthMeters() - other.getLengthMeters())
        < Constants.Arm.Extension.TOLERANCE;
  }

  public boolean atAngleOf(ArmPosition other) {
    return Math.abs(this.getAngleRadians() - other.getAngleRadians())
        < Constants.Arm.Rotation.TOLERANCE;
  }
}
