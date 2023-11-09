package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm.Selector;

public class ToGoal extends Command {

  private final Arm arm;
  private final ArmPosition goal;

  private ArmTrajectory trajectory;

  public ToGoal(Arm arm, ArmPosition goal) {
    addRequirements(arm);

    this.arm = arm;
    this.goal = goal;
  }

  @Override
  public void initialize() {
    arm.unlock(Selector.kBoth);
    trajectory = new ArmTrajectory(arm.getPosition(), goal);

    arm.setGoal(goal);
  }

  @Override
  public void execute() {
    ArmPosition setpoint = trajectory.get();

    if (arm.getPosition().at(setpoint)) {
      setpoint = trajectory.next();
    }

    arm.setSetpoint(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Selector.kBoth);
    arm.lock(Selector.kBoth);
  }

  @Override
  public boolean isFinished() {
    return arm.getPosition().at(goal);
  }
}
