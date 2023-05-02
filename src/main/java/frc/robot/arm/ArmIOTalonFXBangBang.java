// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;

public class ArmIOTalonFXBangBang extends ArmIOTalonFXBase {

  public ArmIOTalonFXBangBang() {
    super();
  }

  @Override
  public void configure() {
    super.configure();

    // Always ensure that your motor controllers are set to "coast" before attempting to control
    // them with a bang-bang controller.
    extensionMotor.setNeutralMode(NeutralMode.Coast);
    rotationMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {
    double position = getExtensionPosition();

    if (position < getExtensionPosition()) {
      setExtensionVoltage(Constants.Arm.Extension.BangBang.INCREASE);
    } else if (position > getExtensionPosition()) {
      setExtensionVoltage(Constants.Arm.Extension.BangBang.DECREASE);
    } else {
      setExtensionDisabled();
    }
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {
    double position = getRotationPosition();

    if (position < angleRadians) {
      setRotationVoltage(Constants.Arm.Rotation.BangBang.INCREASE);
    } else if (position > angleRadians) {
      setRotationVoltage(Constants.Arm.Rotation.BangBang.DECREASE);
    } else {
      setRotationDisabled();
    }
  }
}
