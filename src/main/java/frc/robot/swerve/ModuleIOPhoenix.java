package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

public class ModuleIOPhoenix implements ModuleIO {

  private final SwerveModule module;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public ModuleIOPhoenix(ModuleConfiguration config) {
    module = new SwerveModule(config.getSwerveModuleConstants(), config.can.bus, true);

    ConfigurationApplier.apply(Swerve.AZIMUTH_CONFIG, module.getCANcoder());
    ConfigurationApplier.apply(Swerve.DRIVE_CONFIG, module.getDriveMotor());
    ConfigurationApplier.apply(Swerve.STEER_CONFIG, module.getSteerMotor());
  }

  @Override
  public void update() {}

  @Override
  public void setSetpoint(SwerveModuleState setpoint) {
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
