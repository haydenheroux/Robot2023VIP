// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

public class ArmIOSim implements ArmIO {

    private double extensionLengthMeters;
    private boolean extensionBrakeIsActive;

    private double rotationAngleRadians;
    private boolean rotationBrakeIsActive;

    private final double kMetersPerVolt = 0.01;

    private final double kRadiansPerVolt = 0.01;

    public ArmIOSim() {}

    @Override
    public void configure() {}

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.extensionLengthMeters = extensionLengthMeters;
        inputs.extensionBrakeIsActive = extensionBrakeIsActive;

        inputs.rotationAngleRadians = rotationAngleRadians;
        inputs.rotationBrakeIsActive = rotationBrakeIsActive;
    }

    @Override
    public void setExtensionPosition(double lengthMeters) {
        extensionLengthMeters = lengthMeters;
    }

    @Override
    public void setExtensionSetpoint(double lengthMeters) {
        if (extensionBrakeIsActive) return; // Stop motor
        extensionLengthMeters = lengthMeters;
    }

    @Override
    public void setExtensionVoltage(double volts) {
        if (extensionBrakeIsActive) return; // Stop motor
        extensionLengthMeters += volts * kMetersPerVolt;
    }

    @Override
    public void setExtensionBrake(boolean isActive) {
        extensionBrakeIsActive = isActive;
    }

    @Override
    public void setExtensionDisabled() {
    }

    @Override
    public void setRotationPosition(double angleRadians) {
        rotationAngleRadians = angleRadians;
    }

    @Override
    public void setRotationSetpoint(double angleRadians) {
        if (rotationBrakeIsActive) return; // Stop motor
        rotationAngleRadians = angleRadians;
    }

    @Override
    public void setRotationVoltage(double volts) {
        if (rotationBrakeIsActive) return; // Stop motor
        rotationAngleRadians += volts * kRadiansPerVolt;
    }

    @Override
    public void setRotationBrake(boolean isActive) {
        rotationBrakeIsActive = isActive;
    }

    @Override
    public void setRotationDisabled() {
    }

}
