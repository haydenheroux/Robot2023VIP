package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class TunerConstants {
  private static final Slot0Configs steerGains = new Slot0Configs();

  static {
    steerGains.kP = 50;
    steerGains.kI = 0;
    steerGains.kD = 0.05;
    steerGains.kS = 0;
    steerGains.kV = 1.5;
    steerGains.kA = 0;
  }

  private static final Slot0Configs driveGains = new Slot0Configs();

  static {
    driveGains.kP = 3;
    driveGains.kI = 0;
    driveGains.kD = 0;
    driveGains.kS = 0;
    driveGains.kV = 0;
    driveGains.kA = 0;
  }

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 300.0;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  private static final double kSpeedAt12VoltsMps = 6.0;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.5714285714285716;

  private static final double kDriveGearRatio = 6.746031746031747;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kSteerMotorReversed = true;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withFeedbackSource(
              Constants.USE_PRO
                  ? SwerveModuleSteerFeedbackType.FusedCANcoder
                  : SwerveModuleSteerFeedbackType.RemoteCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withSteerMotorInverted(kSteerMotorReversed);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 3;
  private static final int kFrontLeftSteerMotorId = 1;
  private static final int kFrontLeftEncoderId = 2;
  private static final double kFrontLeftEncoderOffset = -0.676025390625;

  private static final double kFrontLeftXPosInches = 11.375;
  private static final double kFrontLeftYPosInches = 11.375;

  // Front Right
  private static final int kFrontRightDriveMotorId = 6;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 5;
  private static final double kFrontRightEncoderOffset = -0.9482421875;

  private static final double kFrontRightXPosInches = 11.375;
  private static final double kFrontRightYPosInches = -11.375;

  // Back Left
  private static final int kBackLeftDriveMotorId = 13;
  private static final int kBackLeftSteerMotorId = 11;
  private static final int kBackLeftEncoderId = 12;
  private static final double kBackLeftEncoderOffset = -0.449951171875;

  private static final double kBackLeftXPosInches = -11.375;
  private static final double kBackLeftYPosInches = 11.375;

  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 9;
  private static final double kBackRightEncoderOffset = -0.781982421875;

  private static final double kBackRightXPosInches = -11.375;
  private static final double kBackRightYPosInches = -11.375;

  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kFrontLeftXPosInches),
          Units.inchesToMeters(kFrontLeftYPosInches),
          kInvertLeftSide);

  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kFrontRightXPosInches),
          Units.inchesToMeters(kFrontRightYPosInches),
          kInvertRightSide);

  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(kBackLeftXPosInches),
          Units.inchesToMeters(kBackLeftYPosInches),
          kInvertLeftSide);

  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(kBackRightXPosInches),
          Units.inchesToMeters(kBackRightYPosInches),
          kInvertRightSide);
}
