package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.Constants.Physical;
import frc.robot.Constants.Swerve;

public class SwerveFactory {

  private static final SwerveModuleConstantsFactory phoenixFactory =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(Swerve.MK4I.DRIVE_RATIO)
          .withSteerMotorGearRatio(Swerve.MK4I.STEER_RATIO)
          .withWheelRadius(0.5 * Physical.WHEEL_DIAMETER)
          .withSlipCurrent(300.0) // TODO
          .withSteerMotorGains(createSteerMotorGains())
          .withDriveMotorGains(createDriveMotorGains())
          .withSpeedAt12VoltsMps(Physical.MAX_SPEED)
          .withFeedbackSource(
              Constants.USE_PRO
                  ? SwerveModuleSteerFeedbackType.FusedCANcoder
                  : SwerveModuleSteerFeedbackType.RemoteCANcoder)
          .withCouplingGearRatio(Swerve.MK4I.COUPLING_RATIO)
          .withSteerMotorInverted(Swerve.MK4I.IS_STEER_INVERTED);

  public static ModuleIO createModule(ModuleConstants constants) {
    if (Constants.IS_SIMULATION) return new ModuleIOCustom(constants);
    if (Constants.USE_PHOENIX) return new ModuleIOPhoenix(constants);
    return new ModuleIOCustom(constants);
  }

  public static AzimuthEncoderIO createAzimuthEncoder(ModuleConstants constants) {
    if (Constants.IS_SIMULATION) return new AzimuthEncoderIOSim();
    return new AzimuthEncoderIOCANcoder(constants.can.azimuth, constants.offset.getRotations());
  }

  public static CANcoderConfiguration createAzimuthEncoderConfig() {
    return new CANcoderConfiguration();
  }

  public static DriveMotorIO createDriveMotor(ModuleConstants constants) {
    if (Constants.IS_SIMULATION) return new DriveMotorIOSim();
    if (Constants.USE_PRO) return new DriveMotorIOTalonFXPro(constants.can.drive);
    return new DriveMotorIOTalonFXPID(constants.can.drive);
  }

  public static TalonFXConfiguration createDriveMotorConfig() {
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.MotorOutput.Inverted =
        Swerve.MK4I.IS_DRIVE_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.MK4I.DRIVE_RATIO;

    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    return driveMotorConfig;
  }

  public static Slot0Configs createDriveMotorGains() {
    Slot0Configs driveGains = new Slot0Configs();

    driveGains.kP = 0;
    driveGains.kI = 0;
    driveGains.kD = 0;
    driveGains.kS = 0;
    driveGains.kV = 0;
    driveGains.kA = 0;

    return driveGains;
  }

  public static SteerMotorIO createSteerMotor(ModuleConstants constants) {
    if (Constants.IS_SIMULATION) return new SteerMotorIOSim();
    if (Constants.USE_PRO)
      return new SteerMotorIOTalonFXPro(constants.can.steer, constants.can.azimuth);
    return new SteerMotorIOTalonFXPID(constants.can.steer, constants.can.azimuth);
  }

  public static TalonFXConfiguration createSteerMotorConfig() {
    TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();

    steerMotorConfig.MotorOutput.Inverted =
        Swerve.MK4I.IS_STEER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return steerMotorConfig;
  }

  public static Slot0Configs createSteerMotorGains() {
    Slot0Configs steerGains = new Slot0Configs();

    steerGains.kP = 0;
    steerGains.kI = 0;
    steerGains.kD = 0;
    steerGains.kS = 0;
    steerGains.kV = 0;
    steerGains.kA = 0;

    return steerGains;
  }

  public static SwerveModuleConstants createModuleConstants(ModuleConstants constants) {
    return phoenixFactory.createModuleConstants(
        constants.can.steer.id,
        constants.can.drive.id,
        constants.can.azimuth.id,
        constants.offset.getRotations(),
        constants.location.getX(),
        constants.location.getY(),
        Swerve.MK4I.IS_DRIVE_INVERTED);
  }
}
