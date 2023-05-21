package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Physical;

public class ArmConstraints {

  public static boolean isIntersectingGrid(ArmPosition position) {
    Translation2d worldPosition = getWorldPosition(position);

    boolean intersectsMiddleRow =
        worldPosition.getX() >= Physical.MIDDLE_DISTANCE
            && worldPosition.getY() <= Physical.MIDDLE_HEIGHT;
    boolean intersectsTopRow =
        worldPosition.getX() >= Physical.TOP_DISTANCE
            && worldPosition.getY() <= Physical.TOP_HEIGHT;

    return intersectsMiddleRow || intersectsTopRow;
  }

  public static boolean isWithinRuleZone(ArmPosition position) {
    Translation2d worldPosition = getWorldPosition(position);

    final boolean belowCeiling = worldPosition.getY() < Physical.MAX_HEIGHT;
    final boolean aboveFloor = worldPosition.getY() > Physical.MIN_HEIGHT;
    final boolean inG107 =
        worldPosition.getX() < Physical.BUMPER_DISTANCE + Physical.MAX_OUT_LENGTH;

    return belowCeiling && aboveFloor && inG107;
  }

  private static Translation2d getWorldPosition(ArmPosition position) {
    return position.plus(Physical.ARM_SHOULDER);
  }
}
