package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.odometry.Odometry;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
  private final Swerve swerve;

  private final DoubleSupplier forwardsVelocity, sidewaysVelocity, forwardsHeading, sidewaysHeading;
  private final BooleanSupplier align;

  public final PIDController thetaController;

  public Drive(
      Swerve swerve,
      DoubleSupplier forwardsVelocity,
      DoubleSupplier sidewaysVelocity,
      DoubleSupplier forwardsHeading,
      DoubleSupplier sidewaysHeading,
      BooleanSupplier align) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.forwardsVelocity = forwardsVelocity;
    this.sidewaysVelocity = sidewaysVelocity;
    this.forwardsHeading = forwardsHeading;
    this.sidewaysHeading = sidewaysHeading;
    this.align = align;

    thetaController = new PIDController(Theta.KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation2d velocity =
        new Translation2d(forwardsVelocity.getAsDouble(), sidewaysVelocity.getAsDouble())
            .times(Constants.Swerve.MAX_SPEED);
    Translation2d heading =
        new Translation2d(forwardsHeading.getAsDouble(), sidewaysHeading.getAsDouble());

    double omegaRadiansPerSecond = 0.0;

    boolean headingRequested = heading.getNorm() > 0.7;

    if (align.getAsBoolean() && headingRequested) {
      omegaRadiansPerSecond =
          thetaController.calculate(
              Odometry.getInstance().getYaw().getRadians(), heading.getAngle().getRadians());
    } else {
      double percent = MathUtil.clamp(heading.getY(), -1, 1);
      omegaRadiansPerSecond = percent * Constants.Swerve.MAX_ANGULAR_SPEED.getRadians();
    }

    omegaRadiansPerSecond =
        MathUtil.clamp(
            omegaRadiansPerSecond,
            -Constants.Swerve.MAX_ANGULAR_SPEED.getRadians(),
            Constants.Swerve.MAX_ANGULAR_SPEED.getRadians());

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(),
            velocity.getY(),
            omegaRadiansPerSecond,
            Odometry.getInstance().getYaw());

    SwerveModuleState[] setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setSetpoints(setpoints);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
