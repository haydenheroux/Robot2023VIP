package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Intake.SideIntake;

public class SideIntakeMath {

  /**
   * Computes the measure of the angle between beta and the normal of alpha.
   *
   * @param alpha angle whose normal will be calculated.
   * @param beta angle who will be measured.
   * @return the measure of the angle between beta and the normal of alpha.
   */
  private static Rotation2d getAngleRelativeToNormal(Rotation2d alpha, Rotation2d beta) {
    Rotation2d normalAngle = alpha.plus(Rotation2d.fromDegrees(90.0));
    return normalAngle.plus(beta);
  }

  private static double getBiasFor(Rotation2d alpha, Rotation2d beta) {
    Rotation2d relativeAngle = getAngleRelativeToNormal(alpha, beta);
    return getBiasFor(relativeAngle);
  }

  private static double getBiasFor(Rotation2d relativeAngle) {
    return SideIntake.Voltages.RELATIVE_BIAS * relativeAngle.getSin();
  }

  public static double getBias() {
    return getBiasFor(SideIntake.MECHANISM_ANGLE, SideIntake.ACCEPT_ANGLE);
  }
}
