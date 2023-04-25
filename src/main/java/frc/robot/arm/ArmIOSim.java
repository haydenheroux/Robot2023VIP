// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

public class ArmIOSim implements ArmIO {

    private double extensionLengthMeters;
    private double extensionVoltage;
    private boolean extensionBrakeIsActive;
    private boolean extensionIsDisabled;

    private double rotationAngleRadians;
    private double rotationVoltage;
    private boolean rotationBrakeIsActive;
    private boolean rotationIsDisabled;

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
        extensionIsDisabled = false;
        extensionLengthMeters = lengthMeters;
    }

    @Override
    public void setExtensionVoltage(double volts) {
        extensionIsDisabled = false;
        extensionVoltage = volts;
    }

    @Override
    public void setExtensionBrake(boolean isActive) {
        extensionIsDisabled = true;
        extensionBrakeIsActive = isActive;
    }

    @Override
    public void setExtensionDisabled() {
        extensionIsDisabled = true;
    }

    @Override
    public void setRotationPosition(double angleRadians) {
        rotationAngleRadians = angleRadians;
    }

    @Override
    public void setRotationSetpoint(double angleRadians) {
        rotationIsDisabled = false;
        rotationAngleRadians = angleRadians;
    }

    @Override
    public void setRotationVoltage(double volts) {
        rotationIsDisabled = false;
        rotationVoltage = volts;
    }

    @Override
    public void setRotationBrake(boolean isActive) {
        rotationIsDisabled = true;
        rotationBrakeIsActive = isActive;
    }

    @Override
    public void setRotationDisabled() {
        rotationIsDisabled = true;
    }

}
