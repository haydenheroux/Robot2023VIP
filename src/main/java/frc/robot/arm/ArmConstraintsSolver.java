// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;

public class ArmConstraintsSolver {

    public static boolean isBelow(double totalLengthMeters, double totalAngleRadians, double maxHeightMeters) {
        double armHeightMeters = totalLengthMeters * Math.sin(totalAngleRadians) + Constants.Arm.Constraints.HEIGHT_OFFSET;
        return armHeightMeters < maxHeightMeters;
    }

    public static boolean isAbove(double totalLengthMeters, double totalAngleRadians, double minHeightMeters) {
        double armHeightMeters = totalLengthMeters * Math.sin(totalAngleRadians) + Constants.Arm.Constraints.HEIGHT_OFFSET;
        return armHeightMeters > minHeightMeters;
    }

    public static boolean isOutLessThan(double totalLengthMeters, double totalAngleRadians, double maxLengthMeters) {
        double armLengthMeters = totalLengthMeters * Math.cos(totalAngleRadians);
        return armLengthMeters < maxLengthMeters;
    }

    public static boolean isWithinRuleZone(ArmPosition position) {
        double totalLengthMeters = position.extensionLengthMeters + Constants.Arm.Constraints.LENGTH_OFFSET;
        double totalAngleRadians = position.rotationAngleRadians;
        return isBelow(totalLengthMeters, totalAngleRadians, Constants.Arm.Constraints.MAX_HEIGHT) && isAbove(totalLengthMeters, totalAngleRadians, Constants.Arm.Constraints.MIN_HEIGHT) && isOutLessThan(totalLengthMeters, totalAngleRadians, Constants.Arm.Constraints.MAX_OUT_LENGTH);
    }
}
