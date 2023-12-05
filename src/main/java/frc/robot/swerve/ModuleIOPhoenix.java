package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class ModuleIOPhoenix implements ModuleIO {

  private final SwerveModule module;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public ModuleIOPhoenix(ModuleConstants constants) {
    module =
        new SwerveModule(
            SwerveFactory.createModuleConstants(constants),
            constants.can.azimuth.bus,
            Constants.USE_PRO);
  }

  @Override
  public void configure() {}

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
