package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.controllers.feedforward.TelescopingArmFeedforward;
import frc.robot.arm.ArmPosition;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.ModuleConfiguration.ModuleCAN;
import frc.robot.swerve.ModuleConfiguration.ModuleLocation;
import frc.robot.swerve.SwerveMath;

public class Constants {

  /** Time taken to run one iteration of robot code, in seconds. */
  public static final double LOOP_TIME = 0.02;
  /** Number of iterations of robot code that are run per second. */
  public static final double ITERATIONS_PER_SECOND = 1.0 / LOOP_TIME;
  /** Expected nominal operating voltage of robot subsystems, in volts. */
  public static final double NOMINAL_VOLTAGE = 12.0;

  public static class Ports {

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final int PNEUMATICS_MODULE = 30;

    public static final int TELESCOPING_MOTOR = 3;
    public static final int TELESCOPING_BRAKE = 10;

    public static final int PIVOT_MOTOR = 2;
    public static final int PIVOT_BRAKE = 9;

    public static final int CLAW = 7;

    public static final int BOTTOM_ROLLER = 6;
    public static final int TOP_ROLLER = 5;

    public static final ModuleCAN NORTH_WEST = new ModuleCAN(1, 2, 3, "Drivetrain");
    public static final ModuleCAN NORTH_EAST = new ModuleCAN(4, 5, 6, "Drivetrain");
    public static final ModuleCAN SOUTH_EAST = new ModuleCAN(8, 9, 10, "Drivetrain");
    public static final ModuleCAN SOUTH_WEST = new ModuleCAN(11, 12, 13, "Drivetrain");
  }

  /** Physical robot constants including masses, distances, and other critical measurements. */
  public static class Physical {
    /** Mass of the arm, in kilograms. */
    public static final double ARM_MASS = 4.5;
    /** Distance from the arm pivot point ("shoulder") to the floor. */
    public static final Translation2d ARM_SHOULDER =
        new Translation2d(0, Units.inchesToMeters(28.72));
    /** Distance from the arm pivot point to the end of the static section. */
    public static final double ARM_BASE_LENGTH = Units.inchesToMeters(13);
    /** Distance from the base of the claw to the end of the claw. */
    public static final double CLAW_LENGTH = Units.inchesToMeters(12);
    /**
     * Difference between the total length of the arm (distance from the shoulder to end of claw)
     * and the length of the extending section.
     */
    public static final double LENGTH_OFFSET = ARM_BASE_LENGTH + CLAW_LENGTH;
    /**
     * Distance between the center axis of the robot (in line with the arm shoulder) to the edge of
     * the bumpers.
     */
    public static final double BUMPER_DISTANCE = Units.inchesToMeters(14);

    /** Maximum vertical distance from the floor to the end of the arm. */
    public static final double MAX_HEIGHT = Units.feetToMeters(6) + Units.inchesToMeters(6);
    /** Minimum vertical distance from the floor to the end of the arm. */
    public static final double MIN_HEIGHT = Units.inchesToMeters(4);
    /** Maxiumum horizontal distance from the edge of the frame perimeter to the end of the arm. */
    public static final double MAX_OUT_LENGTH = Units.inchesToMeters(48);

    /**
     * Distance from the bumpers to the front of the middle node, plus the distance from the arm to
     * the bumpers.
     */
    public static final double MIDDLE_DISTANCE =
        Units.feetToMeters(1) + Units.inchesToMeters(10.75) + Physical.BUMPER_DISTANCE;
    /** Height from the floor to the top of the middle cone node. */
    public static final double MIDDLE_HEIGHT = Units.feetToMeters(2) + Units.inchesToMeters(10);

    /**
     * Distance from the bumpers to the front of the top node, plus the distance from the arm to the
     * bumpers.
     */
    public static final double TOP_DISTANCE =
        Units.feetToMeters(3) + Units.inchesToMeters(3.75) + Physical.BUMPER_DISTANCE;
    /** Height from the floor to the top of the top cone node. */
    public static final double TOP_HEIGHT = Units.feetToMeters(3) + Units.inchesToMeters(10);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double WHEEL_COF = 1.19;
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
  }

  /** Arm constants relating to the operation of the arm subsystem. */
  public static class Arm {

    /**
     * Telescoping constants. All constants relating to the telescoping motor, telescoping brake,
     * telescoping algorithms, etc.
     */
    public static class Telescoping {
      public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

      static {
        CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CONFIG.Feedback.SensorToMechanismRatio = Telescoping.RATIO;
      }

      /**
       * Change in distance per full revolution of the spool drum. This is usually the circumference
       * of the spool drum, plus a small constant to account for string diameter.
       */
      public static final double DISTANCE_PER_ROTATION = Units.inchesToMeters(1) * Math.PI;
      /** Total gear ratio between the motor and the spool drum. */
      public static final double RATIO = 15.34;

      /** Minimum length of the arm. */
      public static final double MIN_LENGTH = ArmPosition.STOW.getLength();
      /**
       * Maximum length of the telescoping section. Note that this does NOT equal the maximum TOTAL
       * extension length.
       */
      public static final double MAX_LENGTH = Units.feetToMeters(5);

      /** Maximum telescoping section length error. */
      public static final double TOLERANCE = 0.01;

      public static final TelescopingArmFeedforward FEEDFORWARD = new TelescopingArmFeedforward();

      static {
        FEEDFORWARD.kG =
            TelescopingArmFeedforward.telescopingKG(
                -1.41421, new ArmPosition(Physical.ARM_BASE_LENGTH, Rotation2d.fromDegrees(-45)));
      }

      /** Constants for telescoping using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per meter of telescoping length error. */
        public static final double KP = 120;
      }
    }

    /** Pivot constants relating to the pivot motor, pivot brake, pivot algorithms, etc. */
    public static class Pivot {
      public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

      static {
        CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        CONFIG.Feedback.SensorToMechanismRatio = Pivot.RATIO;
      }

      public static final double RATIO = 812.0 / 11.0;

      /** Minimum angle of the arm. */
      public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-45);
      /** Maximum angle of the arm. */
      public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60);

      /** Maximum angle error of the arm. */
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1);

      public static final TelescopingArmFeedforward FEEDFORWARD = new TelescopingArmFeedforward();

      static {
        FEEDFORWARD.kG =
            TelescopingArmFeedforward.pivotKG(
                0.4, new ArmPosition(Physical.ARM_BASE_LENGTH, Rotation2d.fromDegrees(-34.504434)));
      }

      /** Constants for pivoting using a PID control algorithm. */
      public static class PID {
        /** Volts to be applied per rotation of angle error. */
        public static final double KP = 32.4;
      }
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
        /** Voltage for accepting game pieces. */
        public static final double ACCEPTING = -6.0;
        /** Voltage for ejecting game pieces. */
        public static final double EJECTING = 6.0;
        /** Voltage for holding on to a game piece. */
        public static final double HOLDING = -1.0;
      }
    }
  }

  /** Swerve drive subsystem constants. */
  public static class Swerve {
    public static final SwerveModuleConstantsFactory FACTORY = new SwerveModuleConstantsFactory();

    public static final double FRONT_BACK_DISTANCE = Units.inchesToMeters(22.75);
    public static final double LEFT_RIGHT_DISTANCE = Units.inchesToMeters(22.75);

    /** Configuration for the north west swerve module. */
    public static final ModuleConfiguration NORTH_WEST =
        new ModuleConfiguration(true, true).withName("North West");

    static {
      NORTH_WEST.azimuthOffsetRotations = -0.179688;
    }

    /** Configuration for the north east swerve module. */
    public static final ModuleConfiguration NORTH_EAST =
        new ModuleConfiguration(true, false).withName("North East");

    static {
      NORTH_EAST.azimuthOffsetRotations = -0.951904;
    }

    /** Configuration for the south east swerve module. */
    public static final ModuleConfiguration SOUTH_EAST =
        new ModuleConfiguration(false, false).withName("South East");

    static {
      SOUTH_EAST.azimuthOffsetRotations = -0.774568;
    }

    /** Configuration for the south west swerve module. */
    public static final ModuleConfiguration SOUTH_WEST =
        new ModuleConfiguration(false, true).withName("South West");

    static {
      SOUTH_WEST.azimuthOffsetRotations = -0.954346;
    }

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            ModuleLocation.get(true, true),
            ModuleLocation.get(true, false),
            ModuleLocation.get(false, false),
            ModuleLocation.get(false, true));

    /** Maximum speed achievable by the swerve drive, in meters per second. */
    public static final double MAX_SPEED = Units.feetToMeters(20);
    /** Minimum speed before applying dejittering algorithm, in meters per second. */
    public static final double DEJITTER_SPEED = Units.inchesToMeters(4);
    /** Maximum acceleration achivable by the swerve drive, in meters per second per second. */
    public static final double MAX_ACCELERATION = Physical.WHEEL_COF * 9.81;
    /** Maximum angular speed achievable by the swerve drive, in rotations per second. */
    public static final Rotation2d MAX_ANGULAR_SPEED =
        SwerveMath.calculateTheoreticalMaxAngularSpeed(MAX_SPEED, ModuleLocation.get(true, true))
            .times(0.5);
    /** Velocity scalar applied while driving in sniper mode. */
    public static final double SNIPER_SCALAR = 0.25;

    public static final Pigeon2Configuration GYRO_CONFIG = new Pigeon2Configuration();

    static {
      GYRO_CONFIG.Pigeon2Features.EnableCompass = false;
    }

    public static final CANcoderConfiguration AZIMUTH_CONFIG = new CANcoderConfiguration();

    public static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();

    static {
      STEER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      STEER_CONFIG.Slot0.kP = 40;
      STEER_CONFIG.Slot0.kD = 0.0;

      STEER_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;
      STEER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

      STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

      STEER_CONFIG.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

      STEER_CONFIG.Feedback.SensorToMechanismRatio = 150.0 / 7.0;
    }

    public static final Rotation2d STEER_TOLERANCE = Rotation2d.fromDegrees(1.0);

    public static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();

    static {
      DRIVE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // TODO
      DRIVE_CONFIG.Slot0.kP = .00495964340175953;

      // TODO
      DRIVE_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 400;
      DRIVE_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent =
          -1 * DRIVE_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent;

      DRIVE_CONFIG.CurrentLimits.StatorCurrentLimit = 65.0;
      DRIVE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

      DRIVE_CONFIG.Feedback.SensorToMechanismRatio = 6.75;
    }

    public static final double DRIVE_TOLERANCE = 0.1;

    public static class Theta {
      public static final double KP = 24.0;

      public static final Rotation2d SATURATION = MAX_ANGULAR_SPEED;
    }

    public static class Drift {
      public static final double KP = 0.0;

      public static final Rotation2d SATURATION = MAX_ANGULAR_SPEED.times(0.5);
    }
  }
}
