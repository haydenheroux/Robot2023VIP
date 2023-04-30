// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Collections;

public class ArmKinematics {

  private static Translation2d armTranslationRelative(ArmPosition position) {
    double length = position.extensionLengthMeters + Constants.Physical.LENGTH_OFFSET;
    Rotation2d angle = Rotation2d.fromRadians(position.rotationAngleRadians);
    return new Translation2d(length, angle);
  }

  private static Translation2d armTranslation(ArmPosition position) {
    Translation2d offset = new Translation2d(0, Constants.Physical.ARM_SHOULDER_HEIGHT);
    return armTranslationRelative(position).plus(offset);
  }

  private static double calculateHorizontal(double distanceMeters, double angleRadians) {
    double cos = Math.cos(angleRadians);
    if (cos == 0) {
      return Double.POSITIVE_INFINITY;
    }

    return distanceMeters / cos;
  }

  public static boolean isAvoidingGrid(ArmPosition position) {
    ArrayList<Double> maximumLengths = new ArrayList<Double>();

    if (position.rotationAngleRadians <= Constants.Arm.Constraints.HYBRID_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.HYBRID_DISTANCE,
              position.rotationAngleRadians));
    } else if (position.rotationAngleRadians <= Constants.Arm.Constraints.MIDDLE_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MIDDLE_DISTANCE,
              position.rotationAngleRadians));
    }

    maximumLengths.removeIf(x -> x < 0);

    if (maximumLengths.size() == 0) return true;

    double maximumLength = Collections.min(maximumLengths) - Constants.Physical.LENGTH_OFFSET;
    return position.extensionLengthMeters < maximumLength;
  }

  public static boolean isWithinRuleZone(ArmPosition position) {
    final Translation2d translation = armTranslation(position);

    final boolean belowCeiling = translation.getY() < Constants.Arm.Constraints.MAX_HEIGHT;
    final boolean aboveFloor = translation.getY() > Constants.Arm.Constraints.MIN_HEIGHT;
    final boolean inG107 =
        translation.getX()
            < Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MAX_OUT_LENGTH;

    return belowCeiling && aboveFloor && inG107;
  }
}
