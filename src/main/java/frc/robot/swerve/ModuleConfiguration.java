package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleConfiguration {

  public final Translation2d kLocationRelativeToCenterMeters;
  public final double kOffsetAngleRadians;
  public final int kAngleMotorID, kDriveMotorID, kAzimuthEncoderID;
  public final String kCANBus;

  public ModuleConfiguration(
      Translation2d locationRelativeToCenterMeters,
      double offsetAngleRadians,
      int angleMotorID,
      int driveMotorID,
      int azimuthEncoderID,
      String canbus) {
    kLocationRelativeToCenterMeters = locationRelativeToCenterMeters;
    kOffsetAngleRadians = offsetAngleRadians;
    kAngleMotorID = angleMotorID;
    kDriveMotorID = driveMotorID;
    kAzimuthEncoderID = azimuthEncoderID;
    kCANBus = canbus;
  }
}
