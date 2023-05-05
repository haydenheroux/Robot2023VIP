// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.arm.Arm.Type;
import java.util.function.DoubleSupplier;

public class ManualRotation extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier percentSupplier;

  public ManualRotation(Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(arm);

    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void initialize() {
    arm.unlock(Type.kRotation);
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();

    boolean isLeavingBounds = isLeavingBounds(percent);
    boolean isIntersectingGrid = arm.isIntersectingGrid() && percent < 0;

    if (isLeavingBounds || isIntersectingGrid) {
      arm.disable(Type.kRotation);
      arm.lock(Type.kRotation);
    } else {
      arm.unlock(Type.kExtension);
      double volts = percent * Constants.NOMINAL_VOLTAGE;
      arm.setVoltage(Type.kRotation, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Type.kRotation);
    arm.lock(Type.kRotation);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isLeavingBounds(double percent) {
    boolean aboveMax = arm.rotationIsAtMax() && percent > 0;
    boolean belowMin = arm.rotationIsAtMin() && percent < 0;
    return aboveMax || belowMin;
  }
}
