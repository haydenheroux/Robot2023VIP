package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Defines helper methods for math functions commonly used with swerve drives. */
public class SwerveMath {

  /**
   * Calculates the theoretical maximum angular speed achievable by the swerve drive.
   *
   * @param maxSpeedMetersPerSecond the maximum linear speed achievable by the swerve drive.
   * @param farthestModule the module farthest from the pivot point.
   * @return the theoretical maximum angular speed.
   */
  public static Rotation2d calculateTheoreticalMaxAngularSpeed(
      double maxSpeedMetersPerSecond, Translation2d farthestModule) {
    double angularRadius = farthestModule.getNorm();
    return Rotation2d.fromRadians(maxSpeedMetersPerSecond / angularRadius);
  }
}
