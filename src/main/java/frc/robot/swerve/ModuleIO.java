package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    /**
     * Sets the module setpoint.
     * 
     * @param setpoint the setpoint of the module.
     */
    public void setSetpoint(SwerveModuleState setpoint);

    /**
     * Returns true if the module is at the setpoint.
     * 
     * @return
     */
    public boolean atSetpoint();

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
