// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Arm.Constraints;
import frc.robot.Constants.Physical;

public class ArmKinematics {
  public static boolean isIntersectingGrid(ArmPosition position) {
    Translation2d worldArmPosition =
        position.plus(new Translation2d(0, Physical.ARM_SHOULDER_HEIGHT));

    boolean intersectsMiddleRow =
        worldArmPosition.getX() >= Constraints.MIDDLE_DISTANCE
            && worldArmPosition.getY() <= Constraints.MIDDLE_HEIGHT;
    boolean intersectsTopRow =
        worldArmPosition.getX() >= Constraints.TOP_DISTANCE
            && worldArmPosition.getY() <= Constraints.TOP_HEIGHT;

    return intersectsMiddleRow || intersectsTopRow;
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
