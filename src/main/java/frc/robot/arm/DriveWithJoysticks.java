// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class DriveWithJoysticks extends CommandBase {

  private final Arm arm;

  private final DoubleSupplier extensionPercentSupplier, rotationPercentSupplier;

  public DriveWithJoysticks(
      DoubleSupplier extensionPercentSupplier, DoubleSupplier rotationPercentSupplier) {
    arm = Arm.getInstance();
    addRequirements(arm);

    this.extensionPercentSupplier = extensionPercentSupplier;
    this.rotationPercentSupplier = rotationPercentSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.drive(-extensionPercentSupplier.getAsDouble(), -rotationPercentSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
