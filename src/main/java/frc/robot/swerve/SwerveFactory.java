package frc.robot.swerve;

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

  public DriveMotorIO createDriveMotor(ModuleConstants constants) {
    if (isSimulation) return new DriveMotorIOSim();
    return new DriveMotorIOTalonFX(constants.can.drive);
  }

  public SteerMotorIO createSteerMotor(ModuleConstants constants) {
    if (isSimulation) return new SteerMotorIOSim();
    return new SteerMotorIOTalonFX(constants.can.steer, constants.can.azimuth);
  }
}
