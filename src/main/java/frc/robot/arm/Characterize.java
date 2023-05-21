package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm.Selector;

public class Characterize extends CommandBase {

  private final Arm arm;

  private final Selector selector;

  public Characterize(Arm arm, Selector selector) {
    addRequirements(arm);

    this.arm = arm;
    this.selector = selector;
  }

  @Override
  public void initialize() {
    arm.unlock(selector);
  }

  @Override
  public void execute() {
    arm.setVoltage(selector, 0);
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(selector);
    arm.lock(selector);
  }
}
