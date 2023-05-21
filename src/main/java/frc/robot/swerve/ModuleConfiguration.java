package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Swerve;

public class ModuleConfiguration {

  public static class ModuleLocation {
    /*
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
     */
    public static Translation2d get(boolean north, boolean west) {
      final double kNorthSouthCornerDistance = Swerve.FRONT_BACK_DISTANCE / 2;
      final double kWestEastCornerDistance = Swerve.LEFT_RIGHT_DISTANCE / 2;
      return new Translation2d(
          north ? kNorthSouthCornerDistance : -kNorthSouthCornerDistance,
          west ? kWestEastCornerDistance : -kWestEastCornerDistance);
    }
  }

  public static class ModuleCAN {
    public final int angle, azimuth, drive;
    public final String bus;

    public ModuleCAN(int angle, int azimuth, int drive, String bus) {
      this.angle = angle;
      this.azimuth = azimuth;
      this.drive = drive;
      this.bus = bus;
    }

    /*
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
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

  public final ModuleCAN can;
  public Rotation2d azimuthOffset = new Rotation2d();
  public String name = "";
  public final Translation2d location;

  /*
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
   */
  public ModuleConfiguration(boolean north, boolean west) {
    can = ModuleCAN.get(north, west);
    location = ModuleLocation.get(north, west);
  }

  public ModuleConfiguration withAzimuthOffset(Rotation2d azimuthOffset) {
    this.azimuthOffset = azimuthOffset;
    return this;
  }

  public ModuleConfiguration withName(String name) {
    this.name = name;
    return this;
  }
}
