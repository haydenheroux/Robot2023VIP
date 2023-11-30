package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CustomRotation3d extends Rotation3d {

  public CustomRotation3d(Rotation2d roll, Rotation2d pitch, Rotation2d yaw) {
    super(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(getX());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(getY());
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(getZ());
  }
}
