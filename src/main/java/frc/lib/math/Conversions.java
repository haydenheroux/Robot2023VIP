// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
       * @return rotation of mechanism, in radians.
       */
      public static double toRadians(double ticks, double gearRatio) {
        return ticks * (2 * Math.PI / (gearRatio * 4096.0));
      }

      /**
       * @param radians rotation of the mechanism, in radians.
       * @param gearRatio gear ratio of mechanism. Use 1 for CANCoder measurement.
       * @return CANCoder ticks.
       */
      public static double fromRadians(double radians, double gearRatio) {
        return radians / (2 * Math.PI / (gearRatio * 4096.0));
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
       * @return rotation of mechanism, in radians.
       */
      public static double toRadians(double ticks, double gearRatio) {
        return ticks * (2 * Math.PI / (gearRatio * 2048.0));
      }

      /**
       * @param radians rotation of mechanism, in radians.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX ticks.
       */
      public static double fromRadians(double radians, double gearRatio) {
        return radians / (2 * Math.PI / (gearRatio * 2048.0));
      }

      /**
       * @param ticks TalonFX ticks.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return position of mechanism, in meters.
       */
      public static double toMeters(double ticks, double circumference, double gearRatio) {
        return ticks * (circumference / (gearRatio * 2048.0));
      }

      /**
       * @param meters position of mechanism, in meters.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX ticks.
       */
      public static double fromMeters(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
      }
    }

    /** Converts TalonFX velocity units (units / 100 ms, 2048-tick basis) and mechanism velocity. */
    public static class Velocity {
      /**
       * @param ticks TalonFX velocity ticks.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return rotation speed, in revolutions per minute.
       */
      public static double toRPM(double ticks, double gearRatio) {
        double motorRPM = ticks * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
      }

      /**
       * @param rpm rotational speed, in revolutions per minute.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX velocity ticks.
       */
      public static double fromRPM(double rpm, double gearRatio) {
        double motorRPM = rpm * gearRatio;
        double velocityTicks = motorRPM * (2048.0 / 600.0);
        return velocityTicks;
      }

      /**
       * @param velocityTicks TalonFX velocity ticks.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return velocity of mechanism, in meters per second.
       */
      public static double toMPS(double velocityTicks, double circumference, double gearRatio) {
        double wheelRPM = toRPM(velocityTicks, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
      }

      /**
       * @param mps velocity of mechanism, in meters per second.
       * @param circumference circumference of mechanism. Alternatively, position per rotation.
       * @param gearRatio gear ratio of mechanism. Use 1 for TalonFX measurement.
       * @return TalonFX velocity ticks.
       */
      public static double fromMPS(double mps, double circumference, double gearRatio) {
        double wheelRPM = ((mps * 60) / circumference);
        double velocityTicks = fromRPM(wheelRPM, gearRatio);
        return velocityTicks;
      }
    }
  }
}
