package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Swerve;

/** Records the configuration of a swerve module. */
public class ModuleConfiguration {
  /** Defines the location of a swerve module relative to the center of the robot. */
  public static class ModuleLocation {
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
    public static Translation2d get(boolean north, boolean west) {
      final double kNorthSouthCornerDistance = Swerve.FRONT_BACK_DISTANCE / 2;
      final double kWestEastCornerDistance = Swerve.LEFT_RIGHT_DISTANCE / 2;
      return new Translation2d(
          north ? kNorthSouthCornerDistance : -kNorthSouthCornerDistance,
          west ? kWestEastCornerDistance : -kWestEastCornerDistance);
    }
  }

  /**
   * Records the CAN IDs and name of the CAN bus for a swerve module.
   *
   * <p>All CAN-networked devices must share the same CAN bus.
   */
  public static class ModuleCAN {
    public final int steer, azimuth, drive;
    public final String bus;

    /**
     * Constructs a new CAN record for a swerve module.
     *
     * @param angle the CAN ID of an angle motor.
     * @param azimuth the CAN ID of an azimuth encoder.
     * @param drive the CAN ID of a drive motor.
     * @param bus the name of the CAN bus for the swerve module.
     */
    public ModuleCAN(int angle, int azimuth, int drive, String bus) {
      this.steer = angle;
      this.azimuth = azimuth;
      this.drive = drive;
      this.bus = bus;
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
  public double azimuthOffsetRotations = 0.0;
  public String name = "";
  public final Translation2d location;

  /**
   * Constructs a new module configuration for a corner swerve module.
   *
   * @param north true if constructing the configuration for a module on the north side.
   * @param west true if constructing the configuration for a module on the west side.
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Coordinate System</a>
   */
  public ModuleConfiguration(boolean north, boolean west) {
    can = ModuleCAN.get(north, west);
    location = ModuleLocation.get(north, west);
  }

  /**
   * Sets the azimuthOffset record of a module configuration.
   *
   * @param azimuthOffset the value of the azimuthOffset record.
   * @return the module configuration.
   */
  public ModuleConfiguration withAzimuthOffset(double azimuthOffset) {
    this.azimuthOffsetRotations = azimuthOffset;
    return this;
  }

  /**
   * Sets the name record of a module configuration.
   *
   * <p>If the name record is not set and using swerve telemetry, the program will crash because of
   * naming conflicts in Shuffleboard and NetworkTables.
   *
   * @param name the value of the name record.
   * @return the module configuration.
   */
  public ModuleConfiguration withName(String name) {
    this.name = name;
    return this;
  }

  public SwerveModuleConstants getSwerveModuleConstants() {
    return SWERVE_MODULE_CONSTANTS_FACTORY.createModuleConstants(
        can.steer,
        can.drive,
        can.steer,
        Units.rotationsToDegrees(azimuthOffsetRotations),
        location.getX(),
        location.getY(),
        false);
  }
}
