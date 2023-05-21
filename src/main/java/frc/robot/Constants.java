package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.ArmPosition;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveMath;
import java.util.HashMap;

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
    public static final Translation2d ARM_SHOULDER =
        new Translation2d(0, Units.inchesToMeters(28.72));
    /** Distance from the arm pivot point to the end of the static section. */
    public static final double ARM_STATIC_SECTION_LENGTH = Units.inchesToMeters(13);
    /** Distance from the base of the claw to the end of the claw. */
    public static final double CLAW_LENGTH = Units.inchesToMeters(12);
    /**
     * Difference between the total length of the arm (distance from the shoulder to end of claw)
     * and the length of the extending section.
     */
    public static final double LENGTH_OFFSET = ARM_STATIC_SECTION_LENGTH + CLAW_LENGTH;
    /**
     * Distance between the center axis of the robot (in line with the arm shoulder) to the edge of
     * the bumpers.
     */
    public static final double BUMPER_DISTANCE = Units.inchesToMeters(14);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double WHEEL_COF = 1.19;
  }

  /** Arm constants relating to the operation of the arm subsystem. */
  public static class Arm {

    /**
     * Extension constants. All constants relating to the extension motor, extension brake,
     * extension algorithms, etc.
     */
    public static class Extension {

      /**
       * Change in distance per full revolution of the spool drum. This is usually the circumference
       * of the spool drum, plus a small constant to account for string diameter.
       */
      public static final double DISTANCE_PER_ROTATION = Units.inchesToMeters(1) * Math.PI;
      /** Total gear ratio between the motor and the spool drum. */
      public static final double GEAR_RATIO = 15.34;

      /** Minimum length of the extending section. */
      public static final double MIN_LENGTH = 0;
      /**
       * Maximum length of the extending section. Note that this does NOT equal the maximum TOTAL
       * extension length.
       */
      public static final double MAX_LENGTH = Units.feetToMeters(5);

      /** Maximum extending section length error. */
      public static final double TOLERANCE = 0.01;

      public static class Feedforward {
        /** Angle of the arm where KG_MIN and KG_MAX was measured. */
        public static final Rotation2d KG_ANGLE = Rotation2d.fromDegrees(90);

        /** Voltage required to barely overcome gravity, causing positive movement. */
        public static final double KG_MAX = 2 / KG_ANGLE.getSin();
        /** Voltage required to barely not overcome gravity, causing negative moement. */
        public static final double KG_MIN = 2 / KG_ANGLE.getSin();

        /** Voltage required to overcome gravity in both directions. */
        public static final double KG = ((KG_MAX + KG_MIN) / 2.0);
        /** Voltage required to overcome static friction. */
        public static final double KS = KG_MAX - KG_MIN;
      }

      /** Constants for extension using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per meter of extending section length error. */
        public static final double KP = 120;
      }
    }

    /**
     * Rotation constants relating to the rotation motor, rotation brake, rotation algorithms, etc.
     */
    public static class Rotation {
      /** Total gear ratio between the motor and the arm rotation. */
      public static final double GEAR_RATIO = 812.0 / 11.0;

      /** Minimum angle of the arm. */
      public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-45);
      /** Maximum angle of the arm. */
      public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60);

      /** Maximum angle error of the arm. */
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1);

      public static class Feedforward {
        /** Length of the arm where KG_MIN and KG_MAX was measured. */
        public static final double KG_LENGTH = Positions.STOW.getLength();

        /** Voltage required to barely overcome gravity, causing positive movement. */
        public static final double KG_MAX = 0.9709744 / KG_LENGTH;
        /** Voltage required to barely not overcome gravity, causing negative moement. */
        public static final double KG_MIN = 0.9709744 / KG_LENGTH;

        /** Voltage required to overcome gravity in both directions. */
        public static final double KG = (KG_MAX + KG_MIN) / 2.0;
        /** Voltage required to overcome static friction. */
        public static final double KS = KG_MAX - KG_MIN;

        /** Voltage required to overcome the resistive force of the spring. */
        public static final double SPRING_VOLTAGE = 0.0; // TODO
      }

      /** Constants for rotation using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per rotation of angle error. */
        public static final double KP = 32.4;
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
          ArmPosition.fromSensorValues(0, Rotation.MAX_ANGLE.getRotations());
      /**
       * Position for safely extending the arm. The arm is rotated up enough such at any extension
       * will not cause the arm to collide with the grid.
       */
      public static final ArmPosition SAFE =
          ArmPosition.fromSensorValues(0, Rotation2d.fromDegrees(30).getRotations());
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
      /** Current draw thresholds for detecting game pieces. */
      public static class Thresholds {
        public static final double PERIOD = 0.5;
        /** Current draw threshold for a game piece, in amps. */
        public static final double THRESHOLD = 20;
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
      /** Assumed angle of the side intake. Used for calculating speed variation based on angle. */
      public static final Rotation2d MECHANISM_ANGLE = Rotation2d.fromDegrees(45);
      /** Assumed angle for accepting game pieces using the side intake. */
      public static final Rotation2d ACCEPT_ANGLE = Rotation2d.fromDegrees(60);
      /** Assumed angle for ejecting game pieces using the side intake. */
      public static final Rotation2d EJECT_ANGLE = Rotation2d.fromDegrees(30);

      /** Current draw thresholds for detecting game pieces. */
      public static class Thresholds {
        public static final double PERIOD = 0.25;
        /** Current draw threshold for a game piece on the bottom motor, in amps. */
        public static final double BOTTOM_THRESHOLD = 20;
        /** Current draw threshold for a game piece on the top motor, in amps. */
        public static final double TOP_THRESHOLD = 20;
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

  /** Swerve drive subsystem constants. */
  public static class Swerve {
    public static final double FRONT_BACK_DISTANCE = Units.inchesToMeters(22.75);
    public static final double LEFT_RIGHT_DISTANCE = Units.inchesToMeters(22.75);

    /** Configuration for the north west swerve module. */
    public static final ModuleConfiguration NORTH_WEST =
        new ModuleConfiguration(true, true).withName("North West");

    /** Configuration for the north east swerve module. */
    public static final ModuleConfiguration NORTH_EAST =
        new ModuleConfiguration(true, false).withName("North East");

    /** Configuration for the south east swerve module. */
    public static final ModuleConfiguration SOUTH_EAST =
        new ModuleConfiguration(false, false).withName("South East");

    /** Configuration for the south west swerve module. */
    public static final ModuleConfiguration SOUTH_WEST =
        new ModuleConfiguration(false, true).withName("South West");

    /** Maximum speed achievable by the swerve drive, in meters per second. */
    public static final double MAX_SPEED = Units.feetToMeters(20);
    /** Minimum speed before applying dejittering algorithm, in meters per second. */
    public static final double DEJITTER_SPEED = Units.inchesToMeters(4);
    /** Maximum acceleration achivable by the swerve drive, in meters per second per second. */
    public static final double MAX_ACCELERATION = Physical.WHEEL_COF * 9.81;
    /** Maximum angular speed achievable by the swerve drive, in rotations per second. */
    public static final Rotation2d MAX_ANGULAR_SPEED =
        SwerveMath.calculateMaxAngularSpeed(MAX_SPEED, NORTH_WEST);

    /** True if all swerve modules are inverted, false is no swerve modules are inverted. */
    public static final boolean INVERTED = true;

    /** Angle motor constants. */
    public static class Angle {
      /** Proportional gain, in ??? per ???. */
      public static final double KP = 0.075;
      /** Derivative gain, in ??? per ???. */
      public static final double KD = 0.4;
      /** Current draw limit, in amps. */
      public static final double CURRENT_LIMIT = 10.0;
      /** Time to go from zero to full, in seconds. */
      public static final double RAMP_TIME = 0.25;
      /** Gear ratio between the angle motor and the wheel. */
      public static final double GEAR_RATIO = 21.42;
    }

    /** Drive motor constants. */
    public static class Drive {
      /** Proportional gain, in ??? per ???. */
      public static final double KP = 0.0020645;
      /** Derivative gain, in ??? per ???. */
      public static final double KD = 0.0;
      /** Current draw limit, in amps. */
      public static final double CURRENT_LIMIT = 65.0;
      /** Time to go from zero to full, in seconds. */
      public static final double RAMP_TIME = 0.25;
      /** Gear ratio between the drive motor and the wheel. */
      public static final double GEAR_RATIO = 6.75;
    }
  }

  public static class Auto {
    public static final PathConstraints SPEEDS = new PathConstraints(4, 3);
    public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
  }
}
