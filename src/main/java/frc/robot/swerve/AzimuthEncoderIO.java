// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

public interface AzimuthEncoderIO {
    public static class AzimuthEncoderIOValues {
        public double absoluteAngleRadians = 0.0;
    }

    public void configure();

    public void updateValues(AzimuthEncoderIOValues values);
}