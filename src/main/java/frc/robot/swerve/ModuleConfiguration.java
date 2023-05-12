package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleConfiguration {

  public final Translation2d locationRelativeToCenterMeters;
  public final double offsetAngleRadians;

  public ModuleConfiguration(
      Translation2d locationRelativeToCenterMeters, double offsetAngleRadians) {
    this.locationRelativeToCenterMeters = locationRelativeToCenterMeters;
    this.offsetAngleRadians = offsetAngleRadians;
  }
}
