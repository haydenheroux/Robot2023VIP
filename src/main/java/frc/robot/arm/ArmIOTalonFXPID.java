// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;

public class ArmIOTalonFXPID extends ArmIOTalonFXBase {

  private final PIDController extensionPID, rotationPID;

  public ArmIOTalonFXPID() {
    super();

    extensionPID = new PIDController(Extension.PID.KP, 0, 0);
    rotationPID = new PIDController(Rotation.PID.KP, 0, 0);
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {
    extensionPID.setSetpoint(lengthMeters);
    double volts = extensionPID.calculate(getExtensionPosition());
    setExtensionVoltage(volts);
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {
    rotationPID.setSetpoint(angleRadians);
    double volts = rotationPID.calculate(getRotationPosition());
    setRotationVoltage(volts);
  }
}
