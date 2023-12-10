package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

  /** Configure hardware. */
  public void configure();

  /** Synchronously poll hardware. */
  public void update();

  /**
   * Sets the module setpoint.
   *
   * @param setpoint the setpoint of the module.
   * @param force if true, do not optimize the setpoint.
   */
  public void setSetpoint(SwerveModuleState setpoint, boolean force);

  /**
   * Gets the most recently set setpoint
   *
   * @return the most recently set setpoint.
   */
  public SwerveModuleState getSetpoint();

  /**
   * Gets the current state of the module.
   *
   * @return the current state of the module.
   */
  public SwerveModuleState getState();

  /**
   * Gets the current position of the module.
   *
   * @return the current position of the module.
   */
  public SwerveModulePosition getPosition();
}
