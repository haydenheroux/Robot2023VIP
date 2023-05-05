// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.mechanism.SuperstructureMechanism;
import frc.lib.telemetry.TelemetryManager;
import frc.robot.arm.Arm;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;
import frc.robot.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Subsystems
  private final Arm arm;
  private final Compressor compressor;
  private final Claw claw;
  private final SideIntake sideIntake;
  private final Swerve swerve;

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Autonomous routine chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    // Initialize subsystems
    arm = Arm.getInstance();
    compressor = new Compressor(Constants.Pneumatics.CAN_ID, Constants.Pneumatics.MODULE_TYPE);
    claw = Claw.getInstance();
    sideIntake = SideIntake.getInstance();
    swerve = Swerve.getInstance();

    TelemetryManager.getInstance()
        .register(
            arm,
            // compressor,
            claw,
            sideIntake,
            swerve);

    TelemetryManager.getInstance().initializeDashboard();

    SmartDashboard.putData("Mechanism", SuperstructureMechanism.getInstance().getMechanism());

    configureAutonomous();
    configureBindings();
    configureDefaultCommands();
    configureTriggers();
  }

  /** Configures the autonomous chooser with autonomous routines. */
  private void configureAutonomous() {}

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
  }

  /** Configures default commands for each subsystem. */
  public void configureDefaultCommands() {}

  /** Configures triggers for arbitrary events. */
  private void configureTriggers() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
