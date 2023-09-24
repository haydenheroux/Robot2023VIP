package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

public class ModuleIOPhoenix implements ModuleIO {

  private final SwerveModule module;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public ModuleIOPhoenix(ModuleConstants constants) {
    // TODO Add back module-level CAN bus, rather than assuming?
    module =
        new SwerveModule(constants.getSwerveModuleConstants(), constants.can.azimuth.bus, true);
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Swerve.AZIMUTH_CONFIG, module.getCANcoder());
    ConfigurationApplier.apply(Swerve.DRIVE_CONFIG, module.getDriveMotor());
    ConfigurationApplier.apply(Swerve.STEER_CONFIG, module.getSteerMotor());
  }

  @Override
  public void update() {}

  @Override
  public void setSetpoint(SwerveModuleState setpoint) {
    // FIXME Stores non-optimized setpoint
    this.setpoint = setpoint;

    module.apply(setpoint, false);
  }

  @Override
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  @Override
  public SwerveModuleState getState() {
    return module.getCurrentState();
  }

  @Override
  public SwerveModulePosition getPosition() {
    return module.getPosition(true);
  }
}
