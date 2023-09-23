package frc.robot.swerve;

public class SwerveFactory {

  private final boolean isSimulation;
  private final boolean shouldUsePhoenix;

  public SwerveFactory(boolean isSimulation, boolean shouldUsePhoenix) {
    this.isSimulation = isSimulation;
    this.shouldUsePhoenix = shouldUsePhoenix;
  }

  public ModuleIO createModule(ModuleConfiguration config) {
    if (isSimulation) return new ModuleIOCustom(config);
    if (shouldUsePhoenix) return new ModuleIOPhoenix(config);
    return new ModuleIOCustom(config);
  }

  public AzimuthEncoderIO createAzimuthEncoder(ModuleConfiguration config) {
    if (isSimulation) return new AzimuthEncoderIOSim();
    return new AzimuthEncoderIOCANcoder(config.can.azimuth, config.offset.getRotations());
  }

  public DriveMotorIO createDriveMotor(ModuleConfiguration config) {
    if (isSimulation) return new DriveMotorIOSim();
    return new DriveMotorIOTalonFX(config.can.drive);
  }

  public SteerMotorIO createSteerMotor(ModuleConfiguration config) {
    if (isSimulation) return new SteerMotorIOSim();
    return new SteerMotorIOTalonFX(config.can.steer, config.can.azimuth);
  }
}
