package frc.lib.math;

/** Converts between commonly used units. */
public class Conversions {

  public static class General {

    /**
     * @param rotations
     * @param circumference
     * @return
     */
    public static double toMeters(double rotations, double circumference) {
      return rotations * circumference;
    }

    /**
     * @param meters
     * @param circumference
     * @return
     */
    public static double toRotations(double meters, double circumference) {
      return meters / circumference;
    }
  }

  /** Implements conversions for CANCoder units. */
  public static class CANCoder {
    /** Converts between CANCoder position units (4096 tick-basis) to mechanism position. */
    public static class Position {
      /**
       * @param ticks CANCoder ticks.
       * @param gearRatio gear ratio of mechanism. Use 1 for CANCoder measurement.
       * @return rotation of mechanism, in rotations.
       */
      public static double toRotations(double ticks, double gearRatio) {
        double rotations = ticks / 4096;
        return rotations / gearRatio;
      }

      /**
       * @param rotations rotation of the mechanism, in rotations.
       * @param gearRatio gear ratio of mechanism. Use 1 for CANCoder measurement.
       * @return CANCoder ticks.
       */
      public static double fromRotations(double rotations, double gearRatio) {
        return rotations * 4096 * gearRatio;
      }
    }
  }

  /** Implements conversions for TalonFX units. */
  public static class TalonFX {
    /** Converts between TalonFX position units (2048-tick basis) and mechanism position. */
    public static class Position {
      /**
       * @param ticks TalonFX ticks.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return rotation of mechanism, in rotations.
       */
      public static double toRotations(double ticks, double gearRatio) {
        double rotations = ticks / 2048;
        return rotations / gearRatio;
      }

      /**
       * @param rotations rotation of mechanism, in rotations.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX ticks.
       */
      public static double fromRotations(double rotations, double gearRatio) {
        return rotations * 2048 * gearRatio;
      }
    }

    /** Converts TalonFX velocity units (units / 100 ms, 2048-tick basis) and mechanism velocity. */
    public static class Velocity {
      /**
       * @param velocityTicks TalonFX velocity ticks.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return velocity of mechanism, in rotations per second.
       */
      public static double toRPS(double velocityTicks, double gearRatio) {
        double velocityTicksPerSecond = velocityTicks * 10;
        return Position.toRotations(velocityTicksPerSecond, gearRatio);
      }

      /**
       * @param rps velocity of mechanism, in rotations per second.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX velocity ticks.
       */
      public static double fromRPS(double rps, double gearRatio) {
        double velocityTicksPerSecond = Position.fromRotations(rps, gearRatio);
        double velocityTicksPer100ms = velocityTicksPerSecond / 10;
        return velocityTicksPer100ms;
      }
    }
  }
}
