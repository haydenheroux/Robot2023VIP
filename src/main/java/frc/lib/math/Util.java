package frc.lib.math;

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
}
