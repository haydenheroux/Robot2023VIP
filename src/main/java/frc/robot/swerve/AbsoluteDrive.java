// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class AbsoluteDrive extends CommandBase {

  private final Swerve swerve;

  private final DoubleSupplier vX, vY, headingX, headingY;

  public AbsoluteDrive(
      Swerve swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingX = headingX;
    this.headingY = headingY;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation2d velocity =
        new Translation2d(vX.getAsDouble(), vY.getAsDouble()).times(Constants.Swerve.MAX_SPEED);
    Rotation2d heading = new Rotation2d(headingX.getAsDouble(), headingY.getAsDouble());
    swerve.drive(velocity, heading);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
