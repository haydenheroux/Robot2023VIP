// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm.LockType;

public class ToGoal extends CommandBase {

  private final Arm arm;

  private final Arm.State goal;

  public ToGoal(Arm.State goal) {
    arm = Arm.getInstance();
    addRequirements(arm);

    this.goal = goal;
  }

  @Override
  public void initialize() {
    arm.setGoal(goal);
    if (arm.atGoal() == false) {
      arm.unlock(LockType.kBoth);
      arm.enable();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.disable();
    arm.lock(LockType.kBoth);
  }

  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }
}
