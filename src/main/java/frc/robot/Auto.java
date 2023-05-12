package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.intake.Claw;

public class Auto {
  private static final Arm arm = Arm.getInstance();
  private static final Claw claw = Claw.getInstance();

  public static Command toFloor() {
    return arm.toGoal(Constants.Arm.Positions.FLOOR);
  }

  public static Command toStow() {
    return arm.toGoal(Constants.Arm.Positions.STOW);
  }

  public static Command toIntermediate() {
    return arm.toGoal(Constants.Arm.Positions.STOW.withAngle(Constants.Arm.Positions.TOP_ROW));
  }

  public static Command toTop() {
    return arm.toGoal(Constants.Arm.Positions.TOP_ROW);
  }

  public static Command accept() {
    return claw.accept();
  }

  public static Command eject() {
    return Commands.sequence(claw.eject(), Commands.waitSeconds(0.5), claw.disable());
  }

  public static Command scoreTop() {
    return Commands.sequence(toTop(), eject(), toIntermediate());
  }
}
