package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {

  public static boolean approximatelyEqual(double a, double b) {
    return approximatelyEqual(a, b, 0.01);
  }

  public static boolean approximatelyEqual(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  public static boolean approximatelyZero(double a) {
    return approximatelyEqual(a, 0);
  }

  public static boolean approximatelyZero(double a, double epsilon) {
    return approximatelyEqual(a, 0, epsilon);
  }

  public static Rotation2d snapToNearest(Rotation2d angle, Rotation2d multiple) {
    double snappedRadians = snapToNearest(angle.getRadians(), multiple.getRadians());

    return Rotation2d.fromRadians(snappedRadians);
  }

  /**
   * Gets the number snapped to the nearest multiple.
   *
   * @param n the number to snap.
   * @param multiple the multiple to snap to.
   * @return the number snapped to the nearest multiple.
   * @see <a href="https://stackoverflow.com/a/39876671">StackOverflow</a>
   */
  public static double snapToNearest(double n, double multiple) {
    return Math.round(n / multiple) * multiple;
  }
}
