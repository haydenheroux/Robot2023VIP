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
  private final BooleanSupplier align, sniper;

  public final PIDController thetaController;

  public Drive(
      Swerve swerve,
      DoubleSupplier forwardsVelocity,
      DoubleSupplier sidewaysVelocity,
      DoubleSupplier forwardsHeading,
      DoubleSupplier sidewaysHeading,
      BooleanSupplier align,
      BooleanSupplier sniper) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.forwardsVelocity = forwardsVelocity;
    this.sidewaysVelocity = sidewaysVelocity;
    this.forwardsHeading = forwardsHeading;
    this.sidewaysHeading = sidewaysHeading;
    this.align = align;
    this.sniper = sniper;

    thetaController = new PIDController(Theta.KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation2d heading =
        new Translation2d(forwardsHeading.getAsDouble(), sidewaysHeading.getAsDouble());

    double omega = getRequestedOmega(heading, align.getAsBoolean());

    Translation2d velocity =
        new Translation2d(forwardsVelocity.getAsDouble(), sidewaysVelocity.getAsDouble())
            .times(Constants.Swerve.MAX_SPEED);

    ChassisSpeeds chassisSpeeds = getChassisVelocity(velocity, omega, sniper.getAsBoolean());

    SwerveModuleState[] setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setSetpoints(setpoints);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Gets the angular speed for a heading and mode, in radians per second.
   *
   * @param heading the translation of the heading axis.
   * @param aligning true if aligning to a specific heading.
   * @return the angular speed for a heading and mode, in radians per second.
   */
  private double getRequestedOmega(Translation2d heading, boolean aligning) {
    double maxSpeed = Constants.Swerve.MAX_ANGULAR_SPEED.getRadians();

    if (!aligning) {
      double percent = MathUtil.clamp(heading.getY(), -1, 1);
      return maxSpeed * percent;
    }

    if (heading.getNorm() > 0.7) {
      double yawRadians = Odometry.getInstance().getYaw().getRadians();
      double headingRadians = heading.getAngle().getRadians();

      double omega = thetaController.calculate(yawRadians, headingRadians);

      return MathUtil.clamp(omega, -maxSpeed, maxSpeed);
    }

    return 0.0;
  }

  /**
   * Gets the chassis velocity for requested velocities and mode.
   *
   * @param velocity the requested linear velocity, in meters per second.
   * @param omega the requested angular velocity, in radians per second.
   * @param sniping true if driving in robot-centric mode and with reduced speed.
   * @return the chassis velocity.
   */
  private ChassisSpeeds getChassisVelocity(Translation2d velocity, double omega, boolean sniping) {
    if (sniping) {
      velocity = velocity.times(Constants.Swerve.SNIPER_SCALAR);
      omega *= Constants.Swerve.SNIPER_SCALAR;

      return new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        velocity.getX(), velocity.getY(), omega, Odometry.getInstance().getYaw());
  }
}
