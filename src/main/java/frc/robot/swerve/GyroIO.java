// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

public interface GyroIO {
  public static class GyroIOValues {
    public double rollAngleRadians = 0.0;
    public double pitchAngleRadians = 0.0;
    public double yawAngleRadians = 0.0;

    public double xAccelerationMetersPerSecondSquared = 0.0;
    public double yAccelerationMetersPerSecondSquared = 0.0;
    public double zAccelerationMetersPerSecondSquared = 0.0;
  }

  public void configure();

  public void updateValues(GyroIOValues values);

  public void setRollOffset(double rollOffsetRadians);

  public void setPitchOffset(double pitchOffsetRadians);

  public void setYawOffset(double yawOffsetRadians);
}
