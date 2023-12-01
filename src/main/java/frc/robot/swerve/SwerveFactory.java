package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.Swerve;

public class SwerveFactory {

  private final boolean isSimulation;
  private final boolean shouldUsePhoenix;
  private final boolean shouldUsePro;

  public SwerveFactory(boolean isSimulation, boolean shouldUsePhoenix, boolean shouldUsePro) {
    this.isSimulation = isSimulation;
    this.shouldUsePhoenix = shouldUsePhoenix;
    this.shouldUsePro = shouldUsePro;
  }

  public ModuleIO createModule(ModuleConstants constants) {
    if (isSimulation) return new ModuleIOCustom(constants);
    if (shouldUsePhoenix) return new ModuleIOPhoenix(constants);
    return new ModuleIOCustom(constants);
  }

  public AzimuthEncoderIO createAzimuthEncoder(ModuleConstants constants) {
    if (isSimulation) return new AzimuthEncoderIOSim();
    return new AzimuthEncoderIOCANcoder(constants.can.azimuth, constants.offset.getRotations());
  }

  public CANcoderConfiguration createAzimuthEncoderConfig() {
    return new CANcoderConfiguration();
  }

  public DriveMotorIO createDriveMotor(ModuleConstants constants) {
    if (isSimulation) return new DriveMotorIOSim();
    return new DriveMotorIOTalonFX(constants.can.drive);
  }

  public TalonFXConfiguration createDriveMotorConfig() {
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.MotorOutput.Inverted =
        Swerve.MK4I.IS_DRIVE_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.MK4I.DRIVE_RATIO;

    driveMotorConfig.Slot0 = Swerve.DRIVE_GAINS;

    driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    return driveMotorConfig;
  }

  public SteerMotorIO createSteerMotor(ModuleConstants constants) {
    if (isSimulation) return new SteerMotorIOSim();
    return new SteerMotorIOTalonFX(constants.can.steer, constants.can.azimuth);
  }

  public TalonFXConfiguration createSteerMotorConfig() {
    TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();

    steerMotorConfig.MotorOutput.Inverted =
        Swerve.MK4I.IS_STEER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    steerMotorConfig.Feedback.RotorToSensorRatio = Swerve.MK4I.STEER_RATIO;
    steerMotorConfig.Feedback.FeedbackSensorSource =
        shouldUsePro
            ? FeedbackSensorSourceValue.FusedCANcoder
            : FeedbackSensorSourceValue.RemoteCANcoder;

    steerMotorConfig.Slot0 = Swerve.STEER_GAINS;

    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return steerMotorConfig;
  }
}
