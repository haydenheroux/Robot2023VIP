package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.mechanism.Mechanisms;
import frc.lib.telemetry.TelemetryManager;
import frc.robot.Constants.Ports;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.Selector;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Drive;
import frc.robot.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Subsystems
  private final Arm arm;
  private final Compressor compressor;
  private final Claw claw;
  private final SideIntake sideIntake;
  private final Swerve swerve;
  private final Odometry odometry; 

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController pit = new CommandXboxController(5);

  // Autonomous routine chooser
  private final SendableChooser<PathPlannerTrajectory> autoChooser =
      new SendableChooser<PathPlannerTrajectory>();

  public RobotContainer() {
    // Initialize subsystems
    arm = Arm.getInstance();
    compressor = new Compressor(Ports.PNEUMATICS_MODULE, Ports.PNEUMATICS_MODULE_TYPE);
    claw = Claw.getInstance();
    sideIntake = SideIntake.getInstance();
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance(); 

    TelemetryManager.getInstance()
        .register(
            arm,
            // compressor,
            claw,
            sideIntake,
            swerve,
            odometry);

    TelemetryManager.getInstance().initializeDashboard();

    SmartDashboard.putData("Arm Mechanism", Mechanisms.getInstance().getArmMechanism());

    configureAutonomous();
    configureBindings();
    configureDefaultCommands();
    configureTriggers();
  }

  /** Configures the autonomous chooser with autonomous routines. */
  private void configureAutonomous() {
    autoChooser.setDefaultOption("2 Piece Bump", PathPlanner.loadPath("2 Piece Bump", Auto.SPEEDS));
    autoChooser.addOption("2 Piece Clean", PathPlanner.loadPath("2 Piece Clean", Auto.SPEEDS));

    SmartDashboard.putData(autoChooser);
  }

  /** Configures bindings for driver and operator controllers. */
  private void configureBindings() {
    DoubleSupplier extensionAxis =
        () -> MathUtil.applyDeadband(-operator.getRawAxis(XboxController.Axis.kRightY.value), 0.1);
    DoubleSupplier rotationAxis =
        () -> MathUtil.applyDeadband(-operator.getRawAxis(XboxController.Axis.kLeftY.value), 0.1);

    Trigger shouldExtend = new Trigger(() -> extensionAxis.getAsDouble() != 0);
    Trigger shouldRotate = new Trigger(() -> rotationAxis.getAsDouble() != 0);

    shouldExtend.whileTrue(arm.manualExtend(extensionAxis));
    shouldRotate.whileTrue(arm.manualRotate(rotationAxis));

    operator.a().whileTrue(arm.toGoal(Constants.Arm.Positions.FLOOR));
    operator.b().whileTrue(arm.toGoal(Constants.Arm.Positions.MIDDLE_ROW));
    operator.x().whileTrue(arm.toGoal(Constants.Arm.Positions.STOW));
    operator.y().whileTrue(arm.toGoal(Constants.Arm.Positions.TOP_ROW));

    operator.leftTrigger(0.5).onTrue(claw.accept()).onFalse(claw.holdOrDisable());
    operator.rightTrigger(0.5).onTrue(claw.eject()).onFalse(claw.disable());

    operator.leftBumper().onTrue(sideIntake.accept()).onFalse(sideIntake.holdOrDisable());
    operator.rightBumper().onTrue(sideIntake.eject()).onFalse(sideIntake.disable());

    operator
        .start()
        .onTrue(Commands.runOnce(compressor::enableDigital))
        .onFalse(Commands.runOnce(compressor::disable));

    pit.a().whileTrue(arm.characterize(Selector.kPivot));
    pit.b().whileTrue(arm.characterize(Selector.kTelescoping));
    pit.x().whileTrue(arm.toGoal(Constants.Arm.Positions.STOW));
    pit.y().whileTrue(arm.toGoal(Constants.Arm.Positions.SAFE));
  }

  /** Configures default commands for each subsystem. */
  public void configureDefaultCommands() {
    swerve.setDefaultCommand(
        new Drive(
            swerve,
            () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-driver.getLeftX(), 0.1),
            () -> MathUtil.applyDeadband(-driver.getRightY(), 0.1),
            () -> MathUtil.applyDeadband(-driver.getRightX(), 0.1),
            () -> driver.leftTrigger().getAsBoolean()));
  }

  /** Configures triggers for arbitrary events. */
  private void configureTriggers() {}

  public Command getAutonomousCommand() {
    return Auto.BUILDER.fullAuto(autoChooser.getSelected());
  }
}
