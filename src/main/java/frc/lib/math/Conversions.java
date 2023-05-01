// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

public class Conversions {

  public static class CANCoder {
    public static double toRadians(double ticks, double gearRatio) {
      return ticks * (2 * Math.PI / (gearRatio * 4096.0));
    }

    public static double fromRadians(double radians, double gearRatio) {
      return radians / (2 * Math.PI / (gearRatio * 4096.0));
    }
  }

  public static class TalonFX {
    public static double toRadians(double ticks, double gearRatio) {
      return ticks * (2 * Math.PI / (gearRatio * 2048.0));
    }

    public static double fromRadians(double radians, double gearRatio) {
      return radians / (2 * Math.PI / (gearRatio * 2048.0));
    }

    public static double toRPM(double ticks, double gearRatio) {
      double motorRPM = ticks * (600.0 / 2048.0);
      double mechRPM = motorRPM / gearRatio;
      return mechRPM;
    }

    public static double fromRPM(double rpm, double gearRatio) {
      double motorRPM = rpm * gearRatio;
      double velocityTicks = motorRPM * (2048.0 / 600.0);
      return velocityTicks;
    }

    public static double toMPS(double velocityTicks, double circumference, double gearRatio) {
      double wheelRPM = toRPM(velocityTicks, gearRatio);
      double wheelMPS = (wheelRPM * circumference) / 60;
      return wheelMPS;
    }

    public static double fromMPS(double mps, double circumference, double gearRatio) {
      double wheelRPM = ((mps * 60) / circumference);
      double velocityTicks = fromRPM(wheelRPM, gearRatio);
      return velocityTicks;
    }

    public static double toMeters(double ticks, double circumference, double gearRatio) {
      return ticks * (circumference / (gearRatio * 2048.0));
    }

    public static double fromMeters(double meters, double circumference, double gearRatio) {
      return meters / (circumference / (gearRatio * 2048.0));
    }
  }
}
