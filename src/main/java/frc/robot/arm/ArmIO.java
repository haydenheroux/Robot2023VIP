// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

public interface ArmIO {
    public static class ArmIOInputs {
        public double extensionLengthMeters = 0.0;
        public boolean extensionBrakeIsActive = true;

        public double rotationAngleAbsoluteRadians = 0.0;
        public double rotationAngleRelativeRadians = 0.0;
        public boolean rotationBrakeIsActive = true;
    }

    public default void configure() {}

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setExtensionSetpoint(double lengthMeters) {}
    public default void setExtensionVoltage(double volts) {}
    public default void setExtensionBrake(boolean isActive) {}
    public default void setExtensionDisabled(boolean isDisabled) {}

    public default void setRotationSetpoint(double angleRadians) {}
    public default void setRotationVoltage(double volts) {}
    public default void setRotationBrake(boolean isActive) {}
    public default void setRotationDisabled(boolean isDisabled) {}
}
