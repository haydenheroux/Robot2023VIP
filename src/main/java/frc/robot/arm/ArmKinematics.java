// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Collections;

public class ArmKinematics {

  private static double calculateHorizontal(double distanceMeters, double angleRadians) {
    double cos = Math.cos(angleRadians);
    if (cos == 0) {
      return Double.POSITIVE_INFINITY;
    }

    return distanceMeters / cos;
  }

  public static boolean isAvoidingGrid(ArmPosition position) {
    ArrayList<Double> maximumLengths = new ArrayList<Double>();

    if (position.getAngleRadians() <= Constants.Arm.Constraints.HYBRID_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.HYBRID_DISTANCE,
              position.getAngleRadians()));
    } else if (position.getAngleRadians() <= Constants.Arm.Constraints.MIDDLE_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MIDDLE_DISTANCE,
              position.getAngleRadians()));
    }

    maximumLengths.removeIf(x -> x < 0);

    if (maximumLengths.size() == 0) return true;

    double maximumLength = Collections.min(maximumLengths);
    return position.getLengthMeters() < maximumLength;
  }

  public static boolean isWithinRuleZone(ArmPosition position) {
    Translation2d worldArmPosition =
        position.plus(new Translation2d(0, Constants.Physical.ARM_SHOULDER_HEIGHT));

    final boolean belowCeiling = worldArmPosition.getY() < Constants.Arm.Constraints.MAX_HEIGHT;
    final boolean aboveFloor = worldArmPosition.getY() > Constants.Arm.Constraints.MIN_HEIGHT;
    final boolean inG107 =
        worldArmPosition.getX()
            < Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MAX_OUT_LENGTH;

    return belowCeiling && aboveFloor && inG107;
  }
}
