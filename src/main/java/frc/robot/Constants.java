// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.arm.Arm.State;
import frc.robot.arm.ArmPosition;
import frc.robot.swerve.ModuleConfiguration;

public class Constants {

  /** Time taken to run one iteration of robot code, in seconds. */
  public static final double LOOP_TIME = 0.02;
  /** Number of iterations of robot code that are run per second. */
  public static final double ITERATIONS_PER_SECOND = 1.0 / LOOP_TIME;
  /** Expected nominal operating voltage of robot subsystems, in volts. */
  public static final double NOMINAL_VOLTAGE = 12.0;

  /** Physical robot constants including masses, distances, and other critical measurements. */
  public static class Physical {
    /** Mass of the arm, in kilograms. */
    public static final double ARM_MASS = 4.5;
    /** Distance from the arm pivot point ("shoulder") to the floor. */
    public static final double ARM_SHOULDER_HEIGHT = Units.inchesToMeters(28.72);
    /** Distance from the arm pivot point to the end of the static section. */
    public static final double ARM_STATIC_SECTION_LENGTH = Units.inchesToMeters(13);
    /** Distance from the base of the claw to the end of the claw. */
    public static final double CLAW_LENGTH = Units.inchesToMeters(12);
    /**
     * Distance between the center axis of the robot (in line with the arm shoulder) to the edge of
     * the bumpers.
     */
    public static final double BUMPER_DISTANCE = Units.inchesToMeters(14);
  }

  /** Pneumatics constants relating to the pneumatics module. */
  public static class Pneumatics {
    /** Type of the pneumatics module. */
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    /** CAN identifier for the pneumatics module. */
    public static final int CAN_ID = 30;
  }

  /** Arm constants relating to the operation of the arm subsystem. */
  public static class Arm {

    /**
     * Extension constants. All constants relating to the extension motor, extension brake,
     * extension algorithms, etc.
     */
    public static class Extension {
      /**
       * CAN identifier for the extension motor. The extension motor must be assigned this CAN
       * identifier and be on the RoboRIO CAN bus to function.
       */
      public static final int CAN_ID = 3;
      /** Channel of the extension brake solenoid on the pneumatics controller. */
      public static final int BRAKE_CHANNEL = 10;

      /**
       * Change in distance per full revolution of the spool drum. This is usually the circumference
       * of the spool drum, plus a small constant to account for string diameter.
       */
      public static final double DISTANCE_PER_REVOLUTION = Units.inchesToMeters(1) * Math.PI;
      /** Total gear ratio between the motor and the spool drum. */
      public static final double GEAR_RATIO = 15.34;

      /**
       * Difference between the total length of the arm (distance from the shoulder to end of claw)
       * and the length of the mobile section.
       */
      public static final double LENGTH_OFFSET =
          Physical.ARM_STATIC_SECTION_LENGTH + Physical.CLAW_LENGTH;
      /** Minimum length of the mobile section. */
      public static final double MIN_LENGTH = 0;
      /**
       * Maximum length of the mobile section. Note that this does NOT equal the maximum TOTAL
       * extension length.
       */
      public static final double MAX_LENGTH = Units.feetToMeters(5);

      /** The maximum mobile section length error. */
      public static final double TOLERANCE = 0.01;

      /** Constants for extension using a bang-bang control algorithm. */
      public static class BangBang {
        /** Volts to be applied to increase mobile section length. */
        public static final double INCREASE = 10;
        /** Volts to be applied to decrease mobile section length. */
        public static final double DECREASE = -10;
      }

      /** Constants for extension using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per meter of mobile section length error. */
        public static final double KP = 3.6 / 0.1; // 3.6V per 10cm
      }
    }

    /**
     * Rotation constants relating to the rotation motor, rotation brake, rotation algorithms, etc.
     */
    public static class Rotation {
      /**
       * CAN identifier for the rotation motor. The rotation motor must be assigned this CAN
       * identifier and be on the RoboRIO CAN bus to function.
       */
      public static final int CAN_ID = 2;
      /** Channel of the extension brake solenoid on the pneumatics controller. */
      public static final int BRAKE_CHANNEL = 9;

      /** Total gear ratio between the motor and the arm rotation. */
      public static final double GEAR_RATIO = 812.0 / 11.0;

      /** Minimum angle of the arm. */
      public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-45);
      /** Maximum angle of the arm. */
      public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60);

      /** The maximum angle error of the arm. */
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1);

      /** Constants for rotation using a bang-bang control algorithm. */
      public static class BangBang {
        /** Volts to be applied to increase angle of the arm. */
        public static final double INCREASE = 4;
        /** Volts to be applied to decrease angle of the arm. */
        public static final double DECREASE = 0;
      }

      /** Constants for rotation using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per degree of angle error. */
        public static final double KP = 0.5 / Units.degreesToRadians(10); // 0.5V per 10 degrees
      }
    }

    /** Arm motion constraint constants relating to the edge positions, and "danger" zones. */
    public static class Constraints {
      /** Maximum vertical distance from the floor to the end of the arm. */
      public static final double MAX_HEIGHT = Units.feetToMeters(6) + Units.inchesToMeters(6);
      /** Minimum vertical distance from the floor to the end of the arm. */
      public static final double MIN_HEIGHT = Units.inchesToMeters(4);
      /**
       * Maxiumum horizontal distance from the edge of the frame perimeter to the end of the arm.
       */
      public static final double MAX_OUT_LENGTH = Units.inchesToMeters(48);

      /**
       * Distance from the bumpers to the front of the middle node, plus the distance from the arm
       * to the bumpers.
       */
      public static final double MIDDLE_DISTANCE =
          Units.feetToMeters(1) + Units.inchesToMeters(10.75) + Physical.BUMPER_DISTANCE;
      /** Height from the floor to the top of the middle cone node. */
      public static final double MIDDLE_HEIGHT = Units.feetToMeters(2) + Units.inchesToMeters(10);

      /**
       * Distance from the bumpers to the front of the top node, plus the distance from the arm to
       * the bumpers.
       */
      public static final double TOP_DISTANCE =
          Units.feetToMeters(3) + Units.inchesToMeters(3.75) + Physical.BUMPER_DISTANCE;
      /** Height from the floor to the top of the top cone node. */
      public static final double TOP_HEIGHT = Units.feetToMeters(3) + Units.inchesToMeters(10);
    }

    /** Arm positions. */
    public static class Positions {
      /**
       * Position for safely stowing the arm. The arm is fully retracted and rotated as far up as
       * possible.
       */
      public static final ArmPosition STOW =
          ArmPosition.fromState(new State(0, Rotation.MAX_ANGLE));
      /**
       * Position for safely extending the arm. The arm is rotated up enough such at any extension
       * will not cause the arm to collide with the grid.
       */
      public static final ArmPosition ABOVE_GRID =
          ArmPosition.fromState(new State(0, Rotation2d.fromDegrees(30)));
      /** Position for accepting floor game pieces and ejecting game pieces onto the floor. */
      public static final ArmPosition FLOOR =
          new ArmPosition(Units.feetToMeters(2.5), Rotation.MIN_ANGLE);
      /** Position for ejecting game pieces onto the middle row. */
      public static final ArmPosition MIDDLE_ROW =
          new ArmPosition(1.15, Rotation2d.fromDegrees(14));
      /** Position for ejecting game pieces onto the top row. */
      public static final ArmPosition TOP_ROW = new ArmPosition(1.7, Rotation2d.fromDegrees(25));
    }
  }

  /** Intake subsystem(s) constants including current draw thresholds, voltages, etc. */
  public static class Intake {

    /** Claw subsystem constants. */
    public static class Claw {
      /**
       * CAN identifier for the claw motor. The claw motor must be assigned this CAN identifier and
       * be on the RoboRIO CAN bus to function.
       */
      public static final int CAN_ID = 7;

      /** Current draw thresholds for detecting game pieces. */
      public static class Thresholds {
        public static final double PERIOD = 0.1;
        /** Current draw threshold for a game piece, in amps. */
        public static final double THRESHOLD = 60;
      }

      /** Voltages for each state. */
      public static class Voltages {
        /** Voltage for accepting a game piece. */
        public static final double ACCEPTING = -8.0;
        /** Voltage for ejecting a game piece. */
        public static final double EJECTING = 6.0;
        /** Voltage for holding on to a cone. */
        public static final double HOLDING = -1.0;
      }
    }

    /** Side intake subsystem constants. */
    public static class SideIntake {
      /**
       * CAN identifier for the bottom motor. The bottom motor must be assigned this CAN identifier
       * and be on the RoboRIO CAN bus to function.
       */
      public static final int BOTTOM_CAN_ID = 6;
      /**
       * CAN identifier for the top motor. The top motor must be assigned this CAN identifier and be
       * on the RoboRIO CAN bus to function.
       */
      public static final int TOP_CAN_ID = 5;

      /** Assumed angle of the side intake. Used for calculating speed variation based on angle. */
      public static final double MECHANISM_ANGLE = Units.degreesToRadians(45);
      /** Assumed angle for accepting game pieces using the side intake. */
      public static final double ACCEPT_ANGLE = Units.degreesToRadians(60);
      /** Assumed angle for ejecting game pieces using the side intake. */
      public static final double EJECT_ANGLE = Units.degreesToRadians(30);

      /** Current draw thresholds for detecting game pieces. */
      public static class Thresholds {
        public static final double PERIOD = 0.1;
        /** Current draw threshold for a game piece on the bottom motor, in amps. */
        public static final double BOTTOM_THRESHOLD = 30;
        /** Current draw threshold for a game piece on the top motor, in amps. */
        public static final double TOP_THRESHOLD = 30;
      }

      /** Voltages for each state. */
      public static class Voltages {
        /**
         * Scalar for controlling the voltage difference between bottom and top motors, depending on
         * angle.
         */
        public static final double RELATIVE_BIAS = 1.0;
        /** Base voltage for accepting game pieces. */
        public static final double BASE_ACCEPTING = -6.0;
        /** Base voltage for ejecting game pieces. */
        public static final double BASE_EJECTING = 6.0;
        /** Voltage for holding on to a game piece. */
        public static final double HOLDING = -1.0;
      }
    }
  }

  public static class Swerve {

    public static final double MAX_SPEED = Units.feetToMeters(20);

    public static class FrontLeft {
      public static final Translation2d LOCATION =
          new Translation2d(Units.inchesToMeters(-22.75), Units.inchesToMeters(22.75));
      public static final ModuleConfiguration CONFIG = new ModuleConfiguration(LOCATION);
    }

    public static class FrontRight {
      public static final Translation2d LOCATION =
          new Translation2d(Units.inchesToMeters(22.75), Units.inchesToMeters(22.75));
      public static final ModuleConfiguration CONFIG = new ModuleConfiguration(LOCATION);
    }

    public static class BackLeft {
      public static final Translation2d LOCATION =
          new Translation2d(Units.inchesToMeters(-22.75), Units.inchesToMeters(-22.75));
      public static final ModuleConfiguration CONFIG = new ModuleConfiguration(LOCATION);
    }

    public static class BackRight {
      public static final Translation2d LOCATION =
          new Translation2d(Units.inchesToMeters(22.75), Units.inchesToMeters(-22.75));
      public static final ModuleConfiguration CONFIG = new ModuleConfiguration(LOCATION);
    }
  }
}
