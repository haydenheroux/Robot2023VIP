package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.controllers.feedforward.TelescopingArmFeedforward;
import frc.robot.arm.ArmPosition;
import frc.robot.swerve.ModuleConstants;
import frc.robot.swerve.ModuleConstants.ModuleCAN;
import frc.robot.swerve.ModuleConstants.ModuleLocation;
import frc.robot.swerve.SwerveFactory;
import frc.robot.swerve.SwerveMath;
import java.util.List;

public class Constants {

  /** Time taken to run one iteration of robot code, in seconds. */
  public static final double LOOP_TIME = 0.02;
  /** Number of iterations of robot code that are run per second. */
  public static final double ITERATIONS_PER_SECOND = 1.0 / LOOP_TIME;
  /** Expected nominal operating voltage of robot subsystems, in volts. */
  public static final double NOMINAL_VOLTAGE = 12.0;
  /** True if the robot is simulated. */
  public static final boolean IS_SIMULATION = Robot.isSimulation();
  /** True if CTRE's Phoenix library should be used. */
  public static final boolean USE_PHOENIX = true;
  /** True if CTRE's Phoenix pro license is active. */
  public static final boolean USE_PRO = false;

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

    public static final String CANIVORE = "Drivetrain";

    public static final ModuleCAN NORTH_WEST = new ModuleCAN(1, 2, 3, CANIVORE);
    public static final ModuleCAN NORTH_EAST = new ModuleCAN(4, 5, 6, CANIVORE);
    public static final ModuleCAN SOUTH_EAST = new ModuleCAN(8, 9, 10, CANIVORE);
    public static final ModuleCAN SOUTH_WEST = new ModuleCAN(11, 12, 13, CANIVORE);

    public static final int CANDLE = 20;

    public static final String CAMERA_NAME = "main";
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

    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d();

    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    public static final AprilTagFieldLayout APRIL_TAGS =
        new AprilTagFieldLayout(
            List.of(
                new AprilTag(
                    1,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(20.19 + 22.0 * 1),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                    2,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(20.19 + 2.0 * 4),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                    3,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(20.19 + 22.0 * 7),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                    4,
                    new Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(FIELD_WIDTH - Units.inchesToMeters(47.96)),
                        Units.inchesToMeters(27.38),
                        new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                    5,
                    new Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(FIELD_WIDTH - Units.inchesToMeters(47.96)),
                        Units.inchesToMeters(27.38),
                        new Rotation3d())),
                new AprilTag(
                    6,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(20.19 + 22.0 * 7),
                        Units.inchesToMeters(18.22),
                        new Rotation3d())),
                new AprilTag(
                    7,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(20.19 + 22.0 * 4),
                        Units.inchesToMeters(18.22),
                        new Rotation3d())),
                new AprilTag(
                    8,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(20.19 + 22.0 * 1),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()))),
            FIELD_LENGTH,
            FIELD_WIDTH);
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
    public static final SwerveFactory FACTORY = new SwerveFactory(IS_SIMULATION, USE_PHOENIX);

    public static final double FRONT_BACK_DISTANCE = Units.inchesToMeters(22.75);
    public static final double LEFT_RIGHT_DISTANCE = Units.inchesToMeters(22.75);

    public static class MK4I {
      public static final boolean IS_DRIVE_INVERTED = true;
      public static final double DRIVE_RATIO = 6.75;

      public static final boolean IS_STEER_INVERTED = true;
      public static final double STEER_RATIO = 150.0 / 7.0;
    }

    /** Configuration for the north west swerve module. */
    public static final ModuleConstants NORTH_WEST =
        new ModuleConstants(true, true, Rotation2d.fromRotations(-0.179688));
    /** Configuration for the north east swerve module. */
    public static final ModuleConstants NORTH_EAST =
        new ModuleConstants(true, false, Rotation2d.fromRotations(-0.951904));
    /** Configuration for the south east swerve module. */
    public static final ModuleConstants SOUTH_EAST =
        new ModuleConstants(false, false, Rotation2d.fromRotations(-0.774568));
    /** Configuration for the south west swerve module. */
    public static final ModuleConstants SOUTH_WEST =
        new ModuleConstants(false, true, Rotation2d.fromRotations(-0.954346));

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            ModuleLocation.of(true, true),
            ModuleLocation.of(true, false),
            ModuleLocation.of(false, false),
            ModuleLocation.of(false, true));

    /** Maximum speed achievable by the swerve drive, in meters per second. */
    public static final double MAX_SPEED = Units.feetToMeters(12);
    /** Maximum acceleration achivable by the swerve drive, in meters per second per second. */
    public static final double MAX_ACCELERATION = Physical.WHEEL_COF * 9.81;
    /** Maximum angular speed achievable by the swerve drive, in rotations per second. */
    public static final Rotation2d MAX_ANGULAR_SPEED =
        SwerveMath.calculateTheoreticalMaxAngularSpeed(MAX_SPEED, ModuleLocation.furthest())
            .times(0.5);
    /** Velocity scalar applied while driving in sniper mode. */
    public static final double SNIPER_SCALAR = 0.25;

    public static final Pigeon2Configuration GYRO_CONFIG = new Pigeon2Configuration();

    static {
      GYRO_CONFIG.Pigeon2Features.EnableCompass = false;
    }

    public static final CANcoderConfiguration AZIMUTH_CONFIG = FACTORY.createAzimuthEncoderConfig();

    public static final double DRIVE_KP = 8.0;

    public static final TalonFXConfiguration DRIVE_CONFIG =
        FACTORY.createDriveMotorConfig(MK4I.IS_DRIVE_INVERTED, MK4I.DRIVE_RATIO, DRIVE_KP, 40.0);

    public static final double STEER_KP = 48.0; // 48.0 fallback

    public static final TalonFXConfiguration STEER_CONFIG =
        FACTORY.createSteerMotorConfig(MK4I.IS_STEER_INVERTED, MK4I.STEER_RATIO, STEER_KP);

    public static class Theta {
      public static final double KP = 7.0;

      public static final Rotation2d SATURATION = MAX_ANGULAR_SPEED;
    }

    public static class Drift {
      public static final double KP = 2.0;

      public static final Rotation2d SATURATION = MAX_ANGULAR_SPEED.times(0.5);
    }
  }

  public static class Lights {

    public static class Colors {
      public static final Color OFF = Color.kBlack;

      public static final Color RSL_ON = Color.kOrangeRed;
    }
  }
}
