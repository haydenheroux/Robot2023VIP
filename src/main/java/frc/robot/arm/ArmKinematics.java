// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Arm.Constraints;
import frc.robot.Constants.Physical;
import java.util.ArrayList;
import java.util.Collections;

public class ArmKinematics {

  private static double calculateHorizontal(double distanceMeters, Rotation2d angle) {
    double cos = angle.getCos();
    if (cos == 0) return Double.POSITIVE_INFINITY;
    return distanceMeters / cos;
  }

  public static boolean isAvoidingGrid(ArmPosition position) {
    ArrayList<Double> maximumLengths = new ArrayList<Double>();

    if (position.getAngle().getRadians() <= Constraints.HYBRID_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Physical.BUMPER_DISTANCE + Constraints.HYBRID_DISTANCE, position.getAngle()));
    } else if (position.getAngle().getRadians() <= Constraints.MIDDLE_ANGLE) {
      maximumLengths.add(
          calculateHorizontal(
              Physical.BUMPER_DISTANCE + Constraints.MIDDLE_DISTANCE, position.getAngle()));
    }

    maximumLengths.removeIf(x -> x < 0);

    if (maximumLengths.size() == 0) return true;

    double maximumLength = Collections.min(maximumLengths);
    return position.getNorm() < maximumLength;
  }

  public static boolean isWithinRuleZone(ArmPosition position) {
    Translation2d worldArmPosition =
        position.plus(new Translation2d(0, Physical.ARM_SHOULDER_HEIGHT));

    final boolean belowCeiling = worldArmPosition.getY() < Constraints.MAX_HEIGHT;
    final boolean aboveFloor = worldArmPosition.getY() > Constraints.MIN_HEIGHT;
    final boolean inG107 =
        worldArmPosition.getX() < Physical.BUMPER_DISTANCE + Constraints.MAX_OUT_LENGTH;

    return belowCeiling && aboveFloor && inG107;
  }
}
