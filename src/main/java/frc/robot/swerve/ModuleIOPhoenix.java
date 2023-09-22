package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleIOPhoenix implements ModuleIO {

  private final SwerveModule module;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public ModuleIOPhoenix(ModuleConfiguration config) {
    module = new SwerveModule(config.getSwerveModuleConstants(), config.can.bus, true);
  }

  @Override
  public void setSetpoint(SwerveModuleState setpoint) {
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
