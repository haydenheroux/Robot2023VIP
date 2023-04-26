// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

  public static final double LOOP_TIME = 0.02;
  public static final double NOMINAL_VOLTAGE = 12.0;

  public static class Arm {

    public static final double MASS = 4.5;

    public static class Extension {
      public static final double TOLERANCE = 0.01;
      public static final double MIN_LENGTH = 0;
      public static final double MAX_LENGTH = 1.0;
    }

    public static class Rotation {
      public static final double GEAR_RATIO = 812.0 / 11.0;
      public static final double MIN_ANGLE = Units.degreesToRadians(-45);
      public static final double MAX_ANGLE = Units.degreesToRadians(60);

      public static final double TOLERANCE = Units.degreesToRadians(1);
    }
  }
}
