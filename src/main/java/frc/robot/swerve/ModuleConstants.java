package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.hardware.CAN;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Swerve;

/** Records the configuration of a swerve module. */
public class ModuleConstants {
  /** Defines the location of a swerve module relative to the center of the robot. */
  public static class ModuleLocation {
    public static final double FORWARD_TO_CENTER_DISTANCE = Swerve.FRONT_BACK_DISTANCE / 2;
    public static final double SIDE_TO_CENTER_DISTANCE = Swerve.LEFT_RIGHT_DISTANCE / 2;

    /**
     * Gets the location of a corner swerve module.
     *
     * @param north true if constructing the configuration for a module on the north side.
     * @param west true if constructing the configuration for a module on the west side.
     * @return the location of a corner swerve module, relative to the center of the robot, in
     *     meters.
     * @see <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     *     Coordinate System</a>
     */
    public static Translation2d of(boolean north, boolean west) {
      return new Translation2d(
          north ? FORWARD_TO_CENTER_DISTANCE : -FORWARD_TO_CENTER_DISTANCE,
          west ? SIDE_TO_CENTER_DISTANCE : -SIDE_TO_CENTER_DISTANCE);
    }

    /**
     * Gets the location of the furthest swerve moodule.
     *
     * @return the location of the furthest swerve module, relative to the center of the robot, in
     *     meters.
     */
    public static Translation2d furthest() {
      return new Translation2d(FORWARD_TO_CENTER_DISTANCE, SIDE_TO_CENTER_DISTANCE);
    }
  }

  /**
   * Records the CAN IDs and name of the CAN bus for a swerve module.
   *
   * <p>All CAN-networked devices must share the same CAN bus.
   */
  public static class ModuleCAN {
    public final CAN steer, azimuth, drive;

    /**
     * Constructs a new CAN record for a swerve module.
     *
     * @param steer the CAN of a steer motor.
     * @param azimuth the CAN of an azimuth encoder.
     * @param drive the CAN of a drive motor.
     */
    public ModuleCAN(CAN steer, CAN azimuth, CAN drive) {
      this.steer = steer;
      this.azimuth = azimuth;
      this.drive = drive;
    }

    public ModuleCAN(int steer, int azimuth, int drive, String bus) {
      this.steer = new CAN(steer, bus);
      this.azimuth = new CAN(azimuth, bus);
      this.drive = new CAN(drive, bus);
    }

    /**
     * Gets the CAN record of a corner swerve module.
     *
     * @param north true if constructing the configuration for a module on the north side.
     * @param west true if constructing the configuration for a module on the west side.
     * @see <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     *     Coordinate System</a>
     */
    public static ModuleCAN get(boolean north, boolean west) {
      if (north && west) {
        return Ports.NORTH_WEST;
      } else if (north && !west) {
        return Ports.NORTH_EAST;
      } else if (!north && !west) {
        return Ports.SOUTH_EAST;
      } else {
        return Ports.SOUTH_WEST;
      }
    }
  }

  private static final SwerveModuleConstantsFactory SWERVE_MODULE_CONSTANTS_FACTORY =
      new SwerveModuleConstantsFactory();

  public final ModuleCAN can;
  public final Translation2d location;
  public final Rotation2d offset;

  /**
   * Constructs a new module configuration for a corner swerve module.
   *
   * @param north true if constructing the configuration for a module on the north side.
   * @param west true if constructing the configuration for a module on the west side.
   * @param offset offset between azimuth encoder angle and actual angle.
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>
   */
  public ModuleConstants(boolean north, boolean west, Rotation2d offset) {
    can = ModuleCAN.get(north, west);
    location = ModuleLocation.of(north, west);
    this.offset = offset;
  }

  public SwerveModuleConstants getSwerveModuleConstants() {
    return SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
        can.steer.id,
        can.drive.id,
        can.steer.id,
        offset.getDegrees(),
        location.getX(),
        location.getY(),
        false);
  }
}
