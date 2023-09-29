package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveFactory {

  private final boolean isSimulation;
  private final boolean shouldUsePhoenix;

  public SwerveFactory(boolean isSimulation, boolean shouldUsePhoenix) {
    this.isSimulation = isSimulation;
    this.shouldUsePhoenix = shouldUsePhoenix;
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

  public TalonFXConfiguration createDriveMotorConfig(
      boolean inverted, double gearRatio, double kP, double currentLimit) {
    final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    driveMotorConfig.Feedback.SensorToMechanismRatio = gearRatio;

    driveMotorConfig.Slot0.kP = kP;

    if (currentLimit > 0.0) {
      driveMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
      driveMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimit;

      driveMotorConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;
      driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = currentLimit * 1.5;
      driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
      driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    return driveMotorConfig;
  }

  public SteerMotorIO createSteerMotor(ModuleConstants constants) {
    if (isSimulation) return new SteerMotorIOSim();
    return new SteerMotorIOTalonFX(constants.can.steer, constants.can.azimuth);
  }

  public TalonFXConfiguration createSteerMotorConfig(
      boolean inverted, double kP) {
    final TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();

    steerMotorConfig.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    steerMotorConfig.Slot0.kP = kP;

    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return steerMotorConfig;
  }
}
