package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.DelayedBoolean;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmPosition;
import frc.robot.intake.Claw;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Swerve;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

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
    return arm.toGoal(ArmPosition.SCORE_GROUND);
  }

  public static Command toStow() {
    return arm.toGoal(ArmPosition.STOW);
  }

  public static Command toIntermediate() {
    return arm.toGoal(ArmPosition.STOW.withAngle(ArmPosition.SCORE_L3));
  }

  public static Command toTop() {
    return arm.toGoal(ArmPosition.SCORE_L3);
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

  public static Command drive(double vX, double vY) {
    return Commands.runOnce(
        () -> {
          ChassisSpeeds speed = new ChassisSpeeds(vX, vY, 0);

          SwerveModuleState[] setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speed);

          swerve.setSetpoints(setpoints, false);
        },
        swerve);
  }

  private static final double ON_FLAP = -5;
  private static final double TIP_FLAP_SPEED = -2.1;

  public static Command tipFlap() {
    BooleanSupplier onFlap = () -> odometry.getPitch().getDegrees() < ON_FLAP;
    return drive(TIP_FLAP_SPEED, 0).until(onFlap).withTimeout(1.0);
  }

  private static final double ON_PLATFORM_DELAY = 0.4;
  private static final double ON_PLATFORM = -12;
  private static final double ON_PLATFORM_THRESHOLD = 3.0;
  private static final double ON_PLATFORM_SPEED = -1.15;

  public static Command driveOnPlatform() {
    DelayedBoolean onPlatformLatch = new DelayedBoolean(Timer.getFPGATimestamp(), ON_PLATFORM_DELAY);

    // TODO Integrate pitch velocity
    BooleanSupplier maybeOnPlatform =
        () -> Math.abs(ON_PLATFORM - odometry.getPitch().getDegrees()) < ON_PLATFORM_THRESHOLD;
    BooleanSupplier onPlatform =
        () -> onPlatformLatch.update(Timer.getFPGATimestamp(), maybeOnPlatform.getAsBoolean());

    return drive(ON_PLATFORM_SPEED, 0).until(onPlatform).withTimeout(1.0);
  }

  private static final double BALANCED_PLATFORM_THRESHOLD = 3.0;
  private static final double BALANCED_PLATFORM_SPEED = -0.85;

  public static Command balanceOnPlatform() {
    // TODO Integrate pitch velocity
    BooleanSupplier platformBalanced = () -> Math.abs(odometry.getPitch().getDegrees()) < BALANCED_PLATFORM_THRESHOLD;

    return drive(BALANCED_PLATFORM_SPEED, 0).until(platformBalanced).withTimeout(1.0);
  }

  public static Command balance() {
    return Commands.sequence(tipFlap(), driveOnPlatform(), balanceOnPlatform(), swerve.cross());
  }
}
