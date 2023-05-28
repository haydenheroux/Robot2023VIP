package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Positions;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.arm.Arm;
import frc.robot.intake.Claw;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Swerve;
import java.util.HashMap;

public class Auto {
  private static final Arm arm = Arm.getInstance();
  private static final Claw claw = Claw.getInstance();
  private static final Swerve swerve = Swerve.getInstance();
  private static final Odometry odometry = Odometry.getInstance();

  public static final PathConstraints SPEEDS = new PathConstraints(4, 3);
  public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();

  static {
    EVENT_MAP.put("toFloor", toFloor());
    EVENT_MAP.put("toIntermediate", toIntermediate());
    EVENT_MAP.put("toStow", toStow());
    EVENT_MAP.put("accept", accept());
    EVENT_MAP.put("scoreTop", scoreTop());
  }

  public static final SwerveAutoBuilder BUILDER =
      new SwerveAutoBuilder(
          odometry::getPose,
          odometry::setPose,
          Constants.Swerve.KINEMATICS,
          new PIDConstants(1.0, 0, 0),
          new PIDConstants(Theta.KP, 0, 0),
          swerve::setSetpoints,
          Auto.EVENT_MAP,
          true,
          swerve);

  public static Command toFloor() {
    return arm.toGoal(Positions.FLOOR);
  }

  public static Command toStow() {
    return arm.toGoal(Positions.STOW);
  }

  public static Command toIntermediate() {
    return arm.toGoal(Positions.STOW.withAngle(Positions.TOP_ROW));
  }

  public static Command toTop() {
    return arm.toGoal(Positions.TOP_ROW);
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
