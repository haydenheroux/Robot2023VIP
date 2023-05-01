// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.arm.ArmPosition;

public class Constants {

  public static final double LOOP_TIME = 0.02;
  public static final double SAMPLES_PER_SECOND = 1.0 / LOOP_TIME;

  public static final double NOMINAL_VOLTAGE = 12.0;

  public static class Physical {
    public static final double ARM_SHOULDER_HEIGHT = Units.inchesToMeters(28.72);
    public static final double STATIC_LENGTH = Units.inchesToMeters(13);
    public static final double CLAW_LENGTH = Units.inchesToMeters(12);
    public static final double LENGTH_OFFSET = STATIC_LENGTH + CLAW_LENGTH;
    public static final double BUMPER_DISTANCE = Units.inchesToMeters(14);
  }

  public static class Arm {

    public static final double MASS = 4.5;

    public static class Extension {
      public static final int CAN_ID = 3;
      public static final int BRAKE_CHANNEL = 10;

      public static final double DISTANCE_PER_ROTATION = Units.inchesToMeters(1) * Math.PI;
      public static final double GEAR_RATIO = 15.34;

      public static final double MIN_LENGTH = 0;
      public static final double MAX_LENGTH = Units.feetToMeters(5);

      public static final double TOLERANCE = 0.01;

      public static final double KP = 3.6 / 0.1; // 3.6V per 10cm
    }

    public static class Rotation {
      public static final int CAN_ID = 2;
      public static final int BRAKE_CHANNEL = 9;

      public static final double GEAR_RATIO = 812.0 / 11.0;

      public static final double MIN_ANGLE = Units.degreesToRadians(-45);
      public static final double MAX_ANGLE = Units.degreesToRadians(60);

      public static final double TOLERANCE = Units.degreesToRadians(1);

      public static final double KP = 0.5 / Units.degreesToRadians(10); // 0.5V per 10 degrees
    }

    public static class Constraints {
      public static final double MAX_HEIGHT = Units.feetToMeters(6) + Units.inchesToMeters(6);
      public static final double MIN_HEIGHT = Units.inchesToMeters(4);
      public static final double MAX_OUT_LENGTH = Units.inchesToMeters(48);

      public static final double HYBRID_ANGLE = Units.degreesToRadians(5);
      public static final double HYBRID_DISTANCE = Units.inchesToMeters(12);

      public static final double MIDDLE_ANGLE = Units.degreesToRadians(16);
      public static final double MIDDLE_DISTANCE =
          Units.inchesToMeters(22.75) + Constants.Physical.CLAW_LENGTH;
    }

    public static class Setpoints {
      public static final ArmPosition STOWED = new ArmPosition(0.0, Rotation.MAX_ANGLE);
      public static final ArmPosition TOP_ROW = new ArmPosition(1.075, Units.degreesToRadians(25));
      public static final ArmPosition MIDDLE_ROW =
          new ArmPosition(0.55, Units.degreesToRadians(14));
      public static final ArmPosition HYRBID = new ArmPosition(0.05, Rotation.MIN_ANGLE);
      public static final ArmPosition AVOIDING_GRID =
          new ArmPosition(0, Units.degreesToRadians(30));
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
      public static final int CAN_ID = 0;
    }

    public static class SideIntake {
      public static final double MECHANISM_ANGLE = Units.degreesToRadians(45);
      public static final double ACCEPT_ANGLE = Units.degreesToRadians(60);
      public static final double EJECT_ANGLE = Units.degreesToRadians(30);

      public static final double CURRENT_PERIOD = 1.0;
      public static final double BOTTOM_CURRENT_THRESHOLD = 30.0;
      public static final double TOP_CURRENT_THRESHOLD = 30.0;

      public static final double RELATIVE_BIAS_FACTOR = 1.0;
      public static final double BASE_ACCEPTING_VOLTAGE = -10.0;
      public static final double BASE_EJECTING_VOLTAGE = 10.0;
      public static final double HOLDING_VOLTAGE = -8;
      public static final int TOP_CAN_ID = 0;
      public static final int BOTTOM_CAN_ID = 0;
    }
  }
}
