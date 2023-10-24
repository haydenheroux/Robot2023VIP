package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.CustomXboxController;
import frc.lib.mechanism.Mechanisms;
import frc.lib.telemetry.TelemetryManager;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmPosition;
import frc.robot.auto.Auto;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Drive;
import frc.robot.swerve.Swerve;

public class RobotContainer {
  private final Arm arm;
  private final Claw claw;
  private final SideIntake sideIntake;
  private final Swerve swerve;
  private final Odometry odometry;

  private final CustomXboxController driver = new CustomXboxController(0);
  private final CustomXboxController operator = new CustomXboxController(1);

  private final SendableChooser<PathPlannerTrajectory> pathChooser =
      new SendableChooser<PathPlannerTrajectory>();

  public RobotContainer() {
    arm = Arm.getInstance();
    claw = Claw.getInstance();
    sideIntake = SideIntake.getInstance();
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    TelemetryManager.getInstance()
        .register(arm, claw, sideIntake, swerve, odometry)
        .initializeDashboards();

    SmartDashboard.putData("Arm Mechanism", Mechanisms.getInstance().getArmMechanism());

    configureAutonomous();
    configureBindings();
    configureDefaultCommands();
    configureTriggers();
  }

  /** Configures the autonomous chooser with autonomous routines. */
  private void configureAutonomous() {
    pathChooser.setDefaultOption("2 Piece Bump", PathPlanner.loadPath("2 Piece Bump", Auto.SPEEDS));
    pathChooser.addOption("2 Piece Clean", PathPlanner.loadPath("2 Piece Clean", Auto.SPEEDS));

    SmartDashboard.putData(pathChooser);
  }

  /** Configures bindings for driver and operator controllers. */
  private void configureBindings() {
    driver.a().whileTrue(swerve.checkForwards());
    driver.b().whileTrue(swerve.checkSideways());
    driver.x().whileTrue(swerve.cross());
    driver.y().whileTrue(new PrintCommand("TODO"));

    operator.leftY().whileTrue(arm.manualRotate(operator::getLeftY));
    operator.rightY().whileTrue(arm.manualExtend(operator::getRightY));

    operator.a().whileTrue(arm.toGoal(ArmPosition.SCORE_L2));
    operator.b().whileTrue(arm.toGoal(ArmPosition.SCORE_L3));
    operator.x().whileTrue(arm.toGoal(ArmPosition.STOW));
    operator.y().whileTrue(arm.toGoal(ArmPosition.SCORE_GROUND));

    operator.leftTrigger().onTrue(claw.accept()).onFalse(claw.holdOrDisable());
    operator.rightTrigger().onTrue(claw.eject()).onFalse(claw.disable());

    operator.leftBumper().onTrue(sideIntake.accept()).onFalse(sideIntake.holdOrDisable());
    operator.rightBumper().onTrue(sideIntake.eject()).onFalse(sideIntake.disable());
  }

  /** Configures default commands for each subsystem. */
  public void configureDefaultCommands() {
    swerve.setDefaultCommand(
        new Drive(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightY(),
            () -> -driver.getRightX(),
            () -> driver.rightTrigger().getAsBoolean(),
            () -> driver.leftTrigger().getAsBoolean()));
  }

  /** Configures triggers for arbitrary events. */
  private void configureTriggers() {}

  /**
   * Gets the command to run during autonomous.
   *
   * @return the command to run during autonomous.
   */
  public Command getAutonomousCommand() {
    // return Auto.BUILDER.fullAuto(pathChooser.getSelected());
    return Auto.driveOdometryTestY(1.0, 0.5);
  }
}
