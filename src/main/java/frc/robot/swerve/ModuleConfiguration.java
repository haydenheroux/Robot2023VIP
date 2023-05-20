package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ModuleConfiguration {

  public final Translation2d kLocationRelativeToCenterMeters;
  public final Rotation2d kOffsetAngle;
  public final int kAngleMotorID, kDriveMotorID, kAzimuthEncoderID;
  public final String kCANBus;

  public ModuleConfiguration(
      Translation2d locationRelativeToCenterMeters,
      Rotation2d offsetAngle,
      int angleMotorID,
      int driveMotorID,
      int azimuthEncoderID,
      String canbus) {
    kLocationRelativeToCenterMeters = locationRelativeToCenterMeters;
    kOffsetAngle = offsetAngle;
    kAngleMotorID = angleMotorID;
    kDriveMotorID = driveMotorID;
    kAzimuthEncoderID = azimuthEncoderID;
    kCANBus = canbus;
  }
}
