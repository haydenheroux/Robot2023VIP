// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

public interface AngleMotorIO {
    public static class AngleMotorIOValues {
        public double angleRadians = 0.0;
        public double omegaRadiansPerSecond = 0.0;
    }

    public void configure();

    public void updateValues(AngleMotorIOValues values);

    public void setPosition(double angleRadians);

    public void setSetpoint(double angleRadians);

    public void setVoltage(double volts);

    public void setBrake(boolean isActive);
}
