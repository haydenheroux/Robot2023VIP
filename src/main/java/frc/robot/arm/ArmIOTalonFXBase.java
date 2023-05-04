// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Constants.Pneumatics;

public class ArmIOTalonFXBase implements ArmIO {

  protected final WPI_TalonFX extensionMotor, rotationMotor;
  private final Solenoid extensionBrake, rotationBrake;

  public ArmIOTalonFXBase() {
    extensionMotor = new WPI_TalonFX(Extension.CAN_ID);
    extensionBrake =
        new Solenoid(Pneumatics.CAN_ID, Pneumatics.MODULE_TYPE, Extension.BRAKE_CHANNEL);

    rotationMotor = new WPI_TalonFX(Rotation.CAN_ID);
    rotationBrake = new Solenoid(Pneumatics.CAN_ID, Pneumatics.MODULE_TYPE, Rotation.BRAKE_CHANNEL);
  }

  @Override
  public void configure() {
    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    rotationMotor.setInverted(true);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateValues(ArmIOValues values) {
    values.extensionLengthMeters = getExtensionPosition();
    values.extensionBrakeIsActive = getExtensionBrakeIsActive();
    values.rotationAngleRadians = getRotationPosition();
    values.rotationBrakeIsActive = getRotationBrakeIsActive();
  }

  @Override
  public void setExtensionPosition(double lengthMeters) {
    double ticks =
        Conversions.TalonFX.Position.fromMeters(
            lengthMeters, Extension.DISTANCE_PER_REVOLUTION, Extension.GEAR_RATIO);
    extensionMotor.setSelectedSensorPosition(ticks);
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {}

  @Override
  public void setExtensionVoltage(double volts) {
    if (getExtensionBrakeIsActive()) {
      System.out.printf("setExtensionVoltage(%d) while extensionBrakeIsActive", volts);
      extensionMotor.disable();
      return;
    }

    extensionMotor.set(ControlMode.PercentOutput, volts / Constants.NOMINAL_VOLTAGE);
  }

  @Override
  public void setExtensionBrake(boolean isActive) {
    extensionBrake.set(isActive);
  }

  @Override
  public void setExtensionDisabled() {
    setExtensionVoltage(0);
  }

  @Override
  public void setRotationPosition(double angleRadians) {
    double ticks = Conversions.TalonFX.Position.fromRadians(angleRadians, Rotation.GEAR_RATIO);
    rotationMotor.setSelectedSensorPosition(ticks);
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {}

  @Override
  public void setRotationVoltage(double volts) {
    if (getRotationBrakeIsActive()) {
      System.out.printf("setRotationVoltage(%f) while rotationBrakeIsActive", volts);
      rotationMotor.disable();
      return;
    }

    rotationMotor.set(ControlMode.PercentOutput, volts / Constants.NOMINAL_VOLTAGE);
  }

  @Override
  public void setRotationBrake(boolean isActive) {
    rotationBrake.set(isActive);
  }

  @Override
  public void setRotationDisabled() {
    setRotationVoltage(0);
  }

  protected double getExtensionPosition() {
    return Conversions.TalonFX.Position.toMeters(
        extensionMotor.getSelectedSensorPosition(),
        Constants.Arm.Extension.DISTANCE_PER_REVOLUTION,
        Constants.Arm.Extension.GEAR_RATIO);
  }

  protected boolean getExtensionBrakeIsActive() {
    return extensionBrake.get();
  }

  protected double getRotationPosition() {
    return Conversions.TalonFX.Position.toRadians(
        rotationMotor.getSelectedSensorPosition(), Rotation.GEAR_RATIO);
  }

  protected boolean getRotationBrakeIsActive() {
    return rotationBrake.get();
  }
}
