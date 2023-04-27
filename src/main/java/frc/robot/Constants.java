// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

  public static final double LOOP_TIME = 0.02;
  public static final double SAMPLES_PER_SECOND = 1.0 / LOOP_TIME;

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

  public static class Intake {
    public static class Claw {
      public static final double GEAR_RATIO = 7.0;
      public static final double MASS = Units.lbsToKilograms(0.269);
      public static final double RADIUS = Units.inchesToMeters(4.0);

      public static final double CURRENT_PERIOD = 1.0;
      public static final double CONE_CURRENT_THRESHOLD = 30.0;
      public static final double CUBE_CURRENT_THRESHOLD = 20.0;

      public static final double ACCEPTING_VOLTAGE = -12.0;
      public static final double EJECTING_VOLTAGE = 12.0;
      public static final double HOLDING_CONE_VOLTAGE = -8;
      public static final double HOLDING_CUBE_VOLTAGE = -4;
    }
  }
}
