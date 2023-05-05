// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.arm.Arm.Type;
import java.util.function.DoubleSupplier;

public class ManualExtend extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier percentSupplier;

  public ManualExtend(Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(arm);

    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void initialize() {
    arm.unlock(Type.kExtension);
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();

    boolean isLeavingBounds = isLeavingBounds(percent);
    boolean isLeavingRuleZone = isLeavingRuleZone(percent);
    boolean isIntersectingGrid = arm.isIntersectingGrid() && percent > 0;

    if (isLeavingBounds || isLeavingRuleZone || isIntersectingGrid) {
      arm.disable(Type.kExtension);
      arm.lock(Type.kExtension);
    } else {
      arm.unlock(Type.kExtension);
      double volts = percent * Constants.NOMINAL_VOLTAGE;
      arm.setVoltage(Type.kExtension, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Type.kExtension);
    arm.lock(Type.kExtension);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isLeavingBounds(double percent) {
    boolean aboveMax = arm.extensionIsAtMax() && percent > 0;
    boolean belowMin = arm.extensionIsAtMin() && percent < 0;
    return aboveMax || belowMin;
  }

  private boolean isLeavingRuleZone(double percent) {
    return arm.isWithinRuleZone() == false && percent > 0;
  }
}
