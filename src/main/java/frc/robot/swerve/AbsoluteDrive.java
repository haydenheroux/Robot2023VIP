package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AbsoluteDrive extends CommandBase {

  private final Swerve swerve;

  private final DoubleSupplier vX, vY, headingX, headingY;
  private final BooleanSupplier spinToggle;

  public AbsoluteDrive(
      Swerve swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingX,
      DoubleSupplier headingY,
      BooleanSupplier spinToggle) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingX = headingX;
    this.headingY = headingY;
    this.spinToggle = spinToggle;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation2d velocity =
        new Translation2d(vX.getAsDouble(), vY.getAsDouble()).times(Constants.Swerve.MAX_SPEED);
    Translation2d heading = new Translation2d(headingX.getAsDouble(), headingY.getAsDouble());

    final boolean isSpinning = spinToggle.getAsBoolean();
    final boolean isAngleSet = heading.getNorm() > 0.1;

    if (isSpinning) {
      double omegaRadiansPerSecond =
          heading.getAngle().getSin() * Constants.Swerve.MAX_ANGULAR_SPEED.getRadians();
      swerve.drive(velocity, omegaRadiansPerSecond);
    } else if (isAngleSet) {
      swerve.drive(velocity, heading.getAngle());
    } else {
      swerve.drive(velocity, swerve.getYaw());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
