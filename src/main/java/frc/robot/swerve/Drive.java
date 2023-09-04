package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.odometry.Odometry;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Commands the swerve to drive. This command has the capability to drive in robot-relative and
 * field-relative modes, while rotating at a requested angular velocity or aligning to a requested
 * heading.
 *
 * <p>Translation requests are generated using the X axis and the Y axis of a joystick. Angular
 * velocity requests are generated using one axis of another joystick. Heading requests are
 * generated using the angular velocity axis and the other axis of the angular velocity joystick.
 *
 * <p>"Align" mode is disabled by default. If in "align" mode, heading requests are used. Otherwise,
 * angular velocity requests are used. "Align" mode is toggled by a joystick trigger.
 *
 * <p>"Sniper" mode is disabled by default. If in "sniper" mode, robot-relative mode is used and the
 * translation request is reduced, as well as disabling all angular velocity and heading requests.
 * Otherwise, field-relative mode is used, and other requests are enabled. "Sniper" mode is toggled
 * by a joystick triger.
 *
 * <p>Robot-relative mode directly transforms joystick translation requests into speed setpoints. A
 * forward (+X) translation request results in movement in line with the current yaw.
 *
 * <p>Field-relative mode translates joystick translation requests into field-relative speed
 * setpoints. A forward (+X) translation request results in movement in line with the long (X) axis
 * of the field.
 *
 * <p>Angular velocity requests cause the chassis to rotate around the center of the robot at the
 * requested angular velocity. A positive (+) request results in counter-clockwise (CCW) rotation.
 *
 * <p>Heading requests cause the chassis to rotate until at the requested heading. Heading requests
 * are snapped to a nearby multiple to allow for aligning to common angles.
 */
public class Drive extends CommandBase {
  private final Swerve swerve;

  private final DoubleSupplier forwardsVelocity, sidewaysVelocity, forwardsHeading, sidewaysHeading;
  private final BooleanSupplier align, sniper;
  private boolean isDrifting, wasDrifting;
  private double lastSetHeadingRadians;

  public final PIDController driftThetaController, thetaController;

  /**
   * Constructs a new drive command.
   *
   * @param swerve the swerve subsystem to drive with.
   * @param forwardsVelocity the supplier for forwards translation requests.
   * @param sidewaysVelocity the supplier for sideways translation requests.
   * @param forwardsHeading the supplier for the forwards component of heading requests.
   * @param sidewaysHeading the supplier for angular velocity requests and the sideways component of
   *     heading requests.
   * @param align the toggle for "align" mode.
   * @param sniper the toggle for "sniper" mode.
   */
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

    isDrifting = false;
    wasDrifting = false;
    lastSetHeadingRadians = Math.toRadians(0.0);

    driftThetaController = new PIDController(Theta.KP * 4.0, 0, 0);
    driftThetaController.enableContinuousInput(-Math.PI, Math.PI);

    thetaController = new PIDController(Theta.KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isDrifting && !wasDrifting)
      lastSetHeadingRadians = Odometry.getInstance().getRotation().getRadians();

    System.out.println(lastSetHeadingRadians);

    wasDrifting = isDrifting;

    Translation2d heading =
        new Translation2d(forwardsHeading.getAsDouble(), sidewaysHeading.getAsDouble());

    double omega = getRequestedOmega(heading, align.getAsBoolean());

    Translation2d velocity =
        new Translation2d(forwardsVelocity.getAsDouble(), sidewaysVelocity.getAsDouble())
            .times(Constants.Swerve.MAX_SPEED);

    ChassisSpeeds chassisSpeeds = getChassisVelocity(velocity, omega, sniper.getAsBoolean());

    SwerveModuleState[] setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setSetpoints(setpoints, false);
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

    final double kMinRequestedOmegaThreshold = 0.1;

    boolean omegaRequested = Math.abs(heading.getY()) > kMinRequestedOmegaThreshold;

    if (!aligning && omegaRequested) {
      isDrifting = false;

      double percent = MathUtil.clamp(heading.getY(), -1, 1);
      return maxSpeed * percent;
    }

    final double yawRadians = Odometry.getInstance().getRotation().getRadians();

    final double kMinHeadingDisplacement = 0.7;

    if (heading.getNorm() < kMinHeadingDisplacement) {
      isDrifting = true;

      double omega = driftThetaController.calculate(yawRadians, lastSetHeadingRadians);

      return MathUtil.clamp(omega, -maxSpeed, maxSpeed);
    }

    isDrifting = false;

    final double kSnapMultiple = Units.degreesToRadians(45);

    double headingRadians = heading.getAngle().getRadians();
    headingRadians = snapToNearest(headingRadians, kSnapMultiple);
    lastSetHeadingRadians = headingRadians;

    double omega = thetaController.calculate(yawRadians, headingRadians);

    return MathUtil.clamp(omega, -maxSpeed, maxSpeed);
  }

  /**
   * Gets the number snapped to the nearest multiple.
   *
   * @param n the number to snap.
   * @param multiple the multiple to snap to.
   * @return the number snapped to the nearest multiple.
   * @see <a href="https://stackoverflow.com/a/39876671">StackOverflow</a>
   */
  private double snapToNearest(double n, double multiple) {
    return Math.round(n / multiple) * multiple;
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
      return new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        velocity.getX(), velocity.getY(), omega, Odometry.getInstance().getRotation());
  }
}
