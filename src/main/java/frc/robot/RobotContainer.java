// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;
import frc.robot.swerve.Swerve;

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
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    claw = Claw.getInstance();
    sideIntake = SideIntake.getInstance();
    swerve = Swerve.getInstance();

    configureAutonomous();
    configureBindings();
    configureTriggers();
  }

  /** Configures the autonomous chooser with autonomous routines. */
  private void configureAutonomous() {}

  /** Configures bindings for driver and operator controllers. */
  private void configureBindings() {}

  /** Configures triggers for arbitrary events. */
  private void configureTriggers() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
