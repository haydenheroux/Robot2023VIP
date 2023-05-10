// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Arm.Constraints;
import frc.robot.Constants.Physical;

public class ArmKinematics {
  public static boolean isIntersectingGrid(ArmPosition position) {
    Translation2d worldPosition = getWorldPosition(position);

    boolean intersectsMiddleRow =
        worldPosition.getX() >= Constraints.MIDDLE_DISTANCE
            && worldPosition.getY() <= Constraints.MIDDLE_HEIGHT;
    boolean intersectsTopRow =
        worldPosition.getX() >= Constraints.TOP_DISTANCE
            && worldPosition.getY() <= Constraints.TOP_HEIGHT;

    return intersectsMiddleRow || intersectsTopRow;
  }

  public static boolean isWithinRuleZone(ArmPosition position) {
    Translation2d worldPosition = getWorldPosition(position);

    final boolean belowCeiling = worldPosition.getY() < Constraints.MAX_HEIGHT;
    final boolean aboveFloor = worldPosition.getY() > Constraints.MIN_HEIGHT;
    final boolean inG107 =
        worldPosition.getX() < Physical.BUMPER_DISTANCE + Constraints.MAX_OUT_LENGTH;

    return belowCeiling && aboveFloor && inG107;
  }

  private static Translation2d getWorldPosition(ArmPosition position) {
    return position.plus(Physical.ARM_SHOULDER);
  }
}
