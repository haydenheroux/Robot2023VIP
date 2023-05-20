package frc.lib.math;

/** Converts between commonly used units. */
public class Conversions {

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

      /**
       * @param ticks TalonFX ticks.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return position of mechanism, in meters.
       */
      public static double toMeters(double ticks, double circumference, double gearRatio) {
        return circumference * toRotations(ticks, gearRatio);
      }

      /**
       * @param meters position of mechanism, in meters.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX ticks.
       */
      public static double fromMeters(double meters, double circumference, double gearRatio) {
        double rotations = meters / circumference;
        return fromRotations(rotations, gearRatio);
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

      /**
       * @param ticks TalonFX velocity ticks.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return rotation speed, in revolutions per minute.
       */
      public static double toRPM(double ticks, double gearRatio) {
        return toRPS(ticks, gearRatio) * 60;
      }

      /**
       * @param rpm rotational speed, in revolutions per minute.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX velocity ticks.
       */
      public static double fromRPM(double rpm, double gearRatio) {
        double rps = rpm / 60;
        return fromRPS(rps, gearRatio);
      }

      /**
       * @param velocityTicks TalonFX velocity ticks.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return velocity of mechanism, in meters per second.
       */
      public static double toMPS(double velocityTicks, double circumference, double gearRatio) {
        double rps = toRPS(velocityTicks, gearRatio);
        return rps * circumference;
      }

      /**
       * @param mps velocity of mechanism, in meters per second.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX velocity ticks.
       */
      public static double fromMPS(double mps, double circumference, double gearRatio) {
        double rps = mps / circumference;
        return fromRPS(rps, gearRatio);
      }
    }
  }
}
