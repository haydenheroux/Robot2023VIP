// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstraintsSolver {

  private static double calculateHorizontal(double distanceMeters, double angleRadians) {
    double cos = Math.cos(angleRadians);
    if (cos == 0) {
      return Double.POSITIVE_INFINITY;
    }

    return distanceMeters / cos;
  }

  private static double calculateVertical(double distanceMeters, double angleRadians) {
    double sin = Math.sin(angleRadians);
    if (sin == 0) {
      return Double.POSITIVE_INFINITY;
    }

    return distanceMeters / sin;
  }

  public static double calculateMaximumExtensionLength(double angleRadians) {
    ArrayList<Double> maximumLengths = new ArrayList<Double>();

    maximumLengths.add(calculateVertical(Constants.Arm.Constraints.MIN_HEIGHT - Constants.Physical.ARM_SHOULDER_HEIGHT, angleRadians));
    maximumLengths.add(calculateVertical(Constants.Arm.Constraints.MAX_HEIGHT - Constants.Physical.ARM_SHOULDER_HEIGHT, angleRadians));

    maximumLengths.add(calculateHorizontal(Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MAX_OUT_LENGTH, angleRadians));
    
    if (angleRadians <= Constants.Arm.Constraints.HYBRID_ANGLE) {
      maximumLengths.add(calculateHorizontal(Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.HYBRID_DISTANCE, angleRadians));
    } else if (angleRadians <= Constants.Arm.Constraints.MIDDLE_ANGLE) {
      maximumLengths.add(calculateHorizontal(Constants.Physical.BUMPER_DISTANCE + Constants.Arm.Constraints.MIDDLE_DISTANCE, angleRadians));
    }

    maximumLengths.removeIf(x -> x < 0);

    return Collections.min(maximumLengths) - Constants.Physical.LENGTH_OFFSET;
  } 

  public static boolean isWithinRuleZone(ArmPosition position) {
    return position.extensionLengthMeters < calculateMaximumExtensionLength(position.rotationAngleRadians);
  }
}
