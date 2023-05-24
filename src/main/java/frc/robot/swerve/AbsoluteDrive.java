package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.pose.PoseEstimator;
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
    double x = vX.getAsDouble();
    double y = vY.getAsDouble();

    /*
     * https://highlandersfrc.com/Documents/Presentations/Swerve%20Drive%20Presentation.pdf
     */
    x *= Math.sqrt(1 - 0.5 * y * y);
    y *= Math.sqrt(1 - 0.5 * x * x);

    Translation2d velocity = new Translation2d(x, y).times(Constants.Swerve.MAX_SPEED);
    Translation2d heading = new Translation2d(headingX.getAsDouble(), headingY.getAsDouble());

    final boolean spinningRequested = spinToggle.getAsBoolean();
    final boolean headingRequested = heading.getNorm() > 0.1;

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), 0.0, PoseEstimator.getInstance().getYaw());
    SwerveModuleState[] setpoints = swerve.kinematics.toSwerveModuleStates(chassisSpeeds);

    swerve.setSetpoints(setpoints);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
